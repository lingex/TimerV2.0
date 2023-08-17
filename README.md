# TimerV2.0
A kittchen timer, mp3 player. STM32F103RET6, W25Q128, RX8025T, NS4168.  

![image](./Doc/004.jpg)


## Feature
- software decode mp3, i2s output.  
- 16MB spi flash disk.  
- FATFS format.  
- DFU Bootloader.  


## Known issues  
- The USBLC6 (D1) will cost leakage when USB is not connect, can be avoid by disable the 'USB_EN' pin or simply cut the 5' pin of this component.  
- ~~USB disconnect interrupt can't be detected while device in sleep mode, may fix later.~~  
- ~~Sometimes return to main UI from the setting menu without expected, may fix later.~~  
- I2S stream CLK has to be set as a half of the real sample rate.  
- The NS4168 may cost about 2 mA current, not 1 uA provided by the datasheet.  


## Note  
- This mp3 decode solution is a limited demo, don't use for any business application.  
- More info can be found here:  
- TimerV2.0/TimerApp/Middlewares/ST/STM32_Audio/Codecs/SpiritDSP_MP3_Dec/Release_Notes.html  
- TimerV2.0/TimerApp/Middlewares/ST/STM32_Audio/Codecs/SpiritDSP_MP3_Dec/doc/SpiritMP3Dec_UG.pdf  

### About the '.dfu' file:  
- ST DFU device can be flash by DfuSe or STM32CubeProgrammer  
- The STM32CubeProgrammer supports both '.bin' and '.hex' file.  
- DfuSe supports '.dfu' file only, so you have to convert the '.bin' or '.hex' file into a '.dfu'.  
- To generate a '.dfu' file, there are some ways: like DfuFileMgr.exe(provide by ST), and some third-part tools like hex2dfu.exe.  
