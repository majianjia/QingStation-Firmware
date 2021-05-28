# QingStation Firmware 
This repository is dedicated for the C language firmware.

The firmware is developed for QingStation hardware.

- Firmware is based on RT-Thread V4.0.3
- IDE: RT-Thread Studio V2.0

For hardware and more info, please see [the main repository for the QingStation](https://github.com/majianjia/QingStation). 

This firmware has start address of `0x8010000` (`64kB` offset). 
Thus, it needs [QingStation bootloader](https://github.com/majianjia/QingStation-bootloader) to boot up.
Or change it back to `0x8000000` by revising the link file. 

List of third party software packages:
- cJSON
- Madgwick
- ulog_file(RT-Thread)
- minmea
- mymqtt
- at_device(RT-Thread)
- tinycrypt(RT-Thread)
- ~~netutils(RT-Thread)~~ (due to License conflict)

Please see their own license for detail. 

# License
The source code is licensed under Apache-2.0 unless from third party packages.
Please see the LICENSE file for detail. 

# Author
Jianjia Ma 

`majianjia(*at*)live.com`
