################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/FATFS/Target/user_diskio.c \
../Drivers/FATFS/Target/user_diskio_spi.c 

C_DEPS += \
./Drivers/FATFS/Target/user_diskio.d \
./Drivers/FATFS/Target/user_diskio_spi.d 

OBJS += \
./Drivers/FATFS/Target/user_diskio.o \
./Drivers/FATFS/Target/user_diskio_spi.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/FATFS/Target/%.o: ../Drivers/FATFS/Target/%.c Drivers/FATFS/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/MRT_Helpers_f4xx/Inc -I../Drivers/MRT_Iridium_f4xx/Inc -I../Drivers/MRT_ISM330DLC_f4xx/Inc -I../Drivers/MRT_RTOS_f4xx/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/MRT_SDCard_f4xx/Inc -I../../Drivers/FATFS/App -I../../Drivers/FATFS/Target -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

