# STM32F1 ROS Subscriber

## New Features and Remarks
This repository is tested and developed using Nucleo 64 STM32F103ZET6. You should refer to branch stm32-f103rb for further instructions.

1. You may need to use the following to enable the communication and the ros parameter in launch file as well. (ttyACM0 is replaced by ttyUSB0)
```
sudo chmod 777 /dev/ttyUSB0
```

2. The VCP of STM32 is ready thus you may have another communication channel to debug your program.

3. IWDG was initialized with 13.2 second limited.

4. Large power motor drive is used instead of TB6612FNG.

## Hardware
![image](https://github.com/vincent51689453/IC382-ROS-STM32/blob/stm32-f103ze/STM32Board.jpg)