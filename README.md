# STM32-ADXL345
ADXL345 3-Axis Accelerometer Library for STM32. HAL-Compatible.

# How to use this library

-- In ADXL.h

1- Change the HAL library header file according to your uC. Here STM32L1xx and hence #include "stm32l1xx_hal.h" is used.

2- Update the SPI settings. Here HAL library is utilized. If you are using STM32CubeMX, set SPI settings and change the handler here, i.e. hspiX.

3- Set the CS pin and port.

-- In main.c

4- Define an ADXL_InitTypeDef structure and set appropriate elements. Full explanation exists in ADXL.h and ADXL.c files. You can also refer to ADXL345 datasheet. 

5- Define an adxlStatus variable.

6- Add ADXL_Init() functions. Its output should be ADXL_OK, otherwise there is some problems.

7- Use other functions like ADXL_Measure(ON) and ADXL_getAccel() as mentioned in the library files.
