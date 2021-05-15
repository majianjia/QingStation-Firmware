/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-05     RealThread   first version
 */

#include <rtthread.h>
#include <board.h>
#include <drv_common.h>

#ifdef RT_USING_MEMHEAP_AS_HEAP
static struct rt_memheap system_heap;
#endif


RT_WEAK void rt_hw_board_init()
{
    extern void hw_board_init(char *clock_src, int32_t clock_src_freq, int32_t clock_target_freq);

    // OTA

    /* Heap initialization */
#if defined(RT_USING_HEAP)
    // default
    rt_system_heap_init((void *) HEAP_BEGIN, (void *) HEAP_END);

    #ifdef RT_USING_MEMHEAP_AS_HEAP
    rt_memheap_init(&system_heap, "sram2", (void *)SRAM2_BASE, SRAM2_SIZE);
    #endif
#endif

    hw_board_init(BSP_CLOCK_SOURCE, BSP_CLOCK_SOURCE_FREQ_MHZ, BSP_CLOCK_SYSTEM_FREQ_MHZ);

    /* Set the shell console output device */
#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

}
