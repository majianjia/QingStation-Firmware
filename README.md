# QingStation Firmware 
This repository is dedicated for the C language firmware.

- Firmware is based on [RT-Thread V4.0.3](https://github.com/RT-Thread/rt-thread)
- IDE: RT-Thread Studio V2.0

For hardware and more info, please see the QingStation main repo: [https://github.com/majianjia/QingStation](https://github.com/majianjia/QingStation). 

This firmware in default has start address of `0x8010000` (`64kB` offset). 

It is to work with [QingStation bootloader](https://github.com/majianjia/QingStation-bootloader) to boot up.

Change it back to `0x8000000` by revising the link file if you want it be standalone. 
Then change `VECT_TAB_OFFSET` back to `0` in `libraries\CMSIS\Device\ST\STM32L4xx\Source\Templates\system_stm32l4xx.c`

List of third party software packages:
- [cJSON](https://github.com/DaveGamble/cJSON)
- [Madgwick](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- [ulog_file(RT-Thread)](https://github.com/RT-Thread-packages/ulog_file)
- [minmea](https://github.com/kosma/minmea)
- [mymqtt](https://github.com/hichard/mymqtt)
- [at_device(RT-Thread)](https://github.com/RT-Thread-packages/at_device)
- [tinycrypt (RT-Thread)](https://github.com/RT-Thread-packages/tinycrypt)
- ~~netutils(RT-Thread)~~ (due to License conflict)

Please see their own license for detail. 

# License
The source code is licensed under Apache-2.0 unless from third party packages.
Please see the LICENSE file for detail. 

# Author
Jianjia Ma 

`majianjia(*at*)live.com`
