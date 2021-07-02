
/*
 * Copyright (c) 2020-2021, Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2021-04-24     Jianjia Ma       the first version
 */
#include <stdio.h>
#include "stdlib.h"
#include "ctype.h"
#include "string.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <dfs_posix.h>
#include <sys/time.h>

#include <netdb.h>
#include <sys/socket.h>
#include <sys/select.h>
#include "configuration.h"

#define DBG_TAG "ntp"
//#define DBG_LVL DBG_INFO
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


#define NTP_TIMESTAMP_DELTA 2208988800ull

#define LI(packet)   (uint8_t) ((packet.li_vn_mode & 0xC0) >> 6) // (li   & 11 000 000) >> 6
#define VN(packet)   (uint8_t) ((packet.li_vn_mode & 0x38) >> 3) // (vn   & 00 111 000) >> 3
#define MODE(packet) (uint8_t) ((packet.li_vn_mode & 0x07) >> 0) // (mode & 00 000 111) >> 0


typedef struct
{
    uint8_t li_vn_mode;      // Eight bits. li, vn, and mode.
                             // li.   Two bits.   Leap indicator.
                             // vn.   Three bits. Version number of the protocol.
                             // mode. Three bits. Client will pick mode 3 for client.

    uint8_t stratum;         // Eight bits. Stratum level of the local clock.
    uint8_t poll;            // Eight bits. Maximum interval between successive messages.
    uint8_t precision;       // Eight bits. Precision of the local clock.

    uint32_t rootDelay;      // 32 bits. Total round trip delay time.
    uint32_t rootDispersion; // 32 bits. Max error aloud from primary clock source.
    uint32_t refId;          // 32 bits. Reference clock identifier.

    uint32_t refTm_s;        // 32 bits. Reference time-stamp seconds.
    uint32_t refTm_f;        // 32 bits. Reference time-stamp fraction of a second.

    uint32_t origTm_s;       // 32 bits. Originate time-stamp seconds.
    uint32_t origTm_f;       // 32 bits. Originate time-stamp fraction of a second.

    uint32_t rxTm_s;         // 32 bits. Received time-stamp seconds.
    uint32_t rxTm_f;         // 32 bits. Received time-stamp fraction of a second.

    uint32_t txTm_s;         // 32 bits and the most important field the client cares about. Transmit time-stamp seconds.
    uint32_t txTm_f;         // 32 bits. Transmit time-stamp fraction of a second.

} ntp_packet;              // Total: 384 bits or 48 bytes.


int ntp_sync(int argc, char* argv[])
{
    int sockfd, n; // Socket file descriptor and the n return result from writing/reading from the socket.
    int portno = 123; // NTP UDP port number.
    int rslt = 0;
    char* host_name = "us.pool.ntp.org"; // NTP server host-name.
    if(argc == 2)
      host_name = argv[1];
    if(argc == 0)
      host_name = argv[0]; // for direct call.

    // Create and zero out the packet. All 48 bytes worth.
    ntp_packet packet;
    memset(&packet, 0, sizeof( ntp_packet ) );

    // Set the first byte's bits to 00,011,011 for li = 0, vn = 3, and mode = 3. The rest will be left set to zero.
    *(( char * ) &packet + 0 ) = 0x1b; // Represents 27 in base 10 or 00011011 in base 2.

    // Create a UDP socket, convert the host-name to an IP address, set the port number,
    // connect to the server, send the packet, and then read in the return packet.
    struct sockaddr_in serv_addr; // Server address data structure.
    struct hostent *server;      // Server data structure.

    sockfd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP ); // Create a UDP socket.
    if ( sockfd < 0 )
    {
    LOG_E( "ERROR opening socket" );
    return -1;
    }

    server = gethostbyname( host_name ); // Convert URL to IP.

    if ( server == NULL ){
      LOG_E( "ERROR, no such host" );
      rslt = -1;
      goto __exit;
    }


    // Zero out the server address structure.
    memset((char*) &serv_addr, 0, sizeof(serv_addr) );

    serv_addr.sin_family = AF_INET;

    // Copy the server's IP address to the server address structure.
    memcpy((char*) &serv_addr.sin_addr.s_addr, (char*)server->h_addr, server->h_length );

    // Convert the port number integer to network big-endian style and save it to the server address structure.
    serv_addr.sin_port = htons(portno);

    // Call up the server using its IP address and port number.
    if ( connect( sockfd, ( struct sockaddr * ) &serv_addr, sizeof( serv_addr) ) < 0 ){
      LOG_E( "ERROR connecting" );
      rslt = -1;
      goto __exit;
    }

    // Send it the NTP packet it wants. If n == -1, it failed.
    n = write(sockfd, (char* ) &packet, sizeof(ntp_packet) );
    if ( n < 0 ){
      LOG_E( "ERROR writing to socket" );
      rslt = -1;
      goto __exit;
    }

    // try select
//    fd_set readset;
//    FD_ZERO(&readset);
//    FD_SET(sockfd, &readset);
//    int select_rslt;
//    int maxfdp1 = sockfd+1;
//    do{
//        rt_thread_delay(1);
//        select_rslt = select(maxfdp1, &readset, 0, 0, 0);
//    }while(select_rslt == 0);
    // select end

    // Wait and receive the packet back from the server. If n == -1, it failed.
    n = read(sockfd, (char* ) &packet, sizeof(ntp_packet) );

    if ( n < 0 ){
      LOG_E( "ERROR reading from socket" );
      rslt = -1;
      goto __exit;
    }


    // These two fields contain the time-stamp seconds as the packet left the NTP server.
    // The number of seconds correspond to the seconds passed since 1900.
    // ntohl() converts the bit/byte order from the network's to host's "endianness".
    packet.txTm_s = ntohl( packet.txTm_s ); // Time-stamp seconds.
    packet.txTm_f = ntohl( packet.txTm_f ); // Time-stamp fraction of a second.

    // Extract the 32 bits that represent the time-stamp seconds (since NTP epoch) from when the packet left the server.
    // Subtract 70 years worth of seconds from the seconds since 1900.
    // This leaves the seconds since the UNIX epoch of 1970.
    // (1900)------------------(1970)**************************************(Time Packet Left the Server)
    time_t txTm = ( time_t ) ( packet.txTm_s - NTP_TIMESTAMP_DELTA );

    // update RTC
    struct tm *tnow = localtime(&txTm);
    set_date(tnow->tm_year + 1900, tnow->tm_mon + 1, tnow->tm_mday);
    tnow = localtime(&txTm);
    set_time(tnow->tm_hour, tnow->tm_min, tnow->tm_sec);

    LOG_I("NTP Time synced: %s", ctime( ( const time_t* ) &txTm ) );

    __exit:
    if (sockfd >= 0)
        closesocket(sockfd);

    return rslt;
}
MSH_CMD_EXPORT(ntp_sync, synchronise ntp)

void thread_ntp(void * parameter)
{
    ntp_config_t* cfg;
    while(!is_system_cfg_valid()) rt_thread_delay(10);
    cfg = &system_config.ntp;

    if(!cfg->is_enable)
        return;

    rt_thread_delay(cfg->startup_delay * RT_TICK_PER_SECOND);

    while(1)
    {
        for(int i=0; i<sizeof(cfg->server)/sizeof(cfg->server[0]); i++)
        {
            LOG_I("sync time from: %s", cfg->server[i]);
            char * addr[1];
            addr[0] = &(cfg->server[i][0]);
            if(!ntp_sync(0, addr))
                break;
        }
        rt_thread_delay(cfg->update_period * RT_TICK_PER_SECOND);
    }
}

int thread_ntp_init()
{
    rt_thread_t tid;
    tid = rt_thread_create("ntp", thread_ntp, NULL, 1366, 10, 10);
    if(tid)
        rt_thread_startup(tid);
    return 0;
}
INIT_APP_EXPORT(thread_ntp_init);

