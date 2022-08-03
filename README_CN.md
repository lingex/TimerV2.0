# TimerV2.0
厨房定时器, 软解MP3. STM32F103RET6, W25Q128, RX8025T, NS4168.  

![image](./Doc/004.jpg)


# 特性
*软解 MP3, i2s 输出到功放(NS4168).  
*16MB spi flash 用于存储配置和音乐文件.  
*FATFS 文件系统.  
*DFU Bootloader.  


# 已知问题  
*在 USB 未连接时 USBLC6 (D1) 会导致漏电, 可通过软件禁止 'USB_EN' 引脚来避免，或者直接把此元件的第5引脚剪掉(ESD防护特性可能有所损失).  
~~*休眠状态拔掉 USB 线不能立即触发中断，可能在晚些时候修复(不修问题也不大).~~  
~~*在菜单界面有时会意外退出到主页面，可能在晚些时候修复.~~  
*I2S 时钟频率需要设置为实际音频采样率的一半播放才正常.  


# 注意  
这个 MP3 软解方案是一个 limited demo, 请勿用于任何商业用途.  
更多信息可以查看:  
TimerV2.0/TimerApp/Middlewares/ST/STM32_Audio/Codecs/SpiritDSP_MP3_Dec/Release_Notes.html  
TimerV2.0/TimerApp/Middlewares/ST/STM32_Audio/Codecs/SpiritDSP_MP3_Dec/doc/SpiritMP3Dec_UG.pdf  

关于 '.dfu' 格式文件:  
ST DFU 设备支持使用 DfuSe 或者 STM32CubeProgrammer 进行烧录  
STM32CubeProgrammer 支持 '.bin' 和 '.hex' 格式.  
DfuSe 仅支持 '.dfu' 格式, 所以你需要把 '.bin' 或者 '.hex' 格式的文件转换为 '.dfu'.  
生成 '.dfu' 文件的有多种方法: 如 DfuFileMgr.exe (ST 提供), 或者一些第三方工具如 hex2dfu.exe.  