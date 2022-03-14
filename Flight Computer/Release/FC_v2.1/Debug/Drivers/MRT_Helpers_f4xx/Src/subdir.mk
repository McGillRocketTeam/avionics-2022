################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.c \
../Drivers/MRT_Helpers_f4xx/Src/w25qxx.c 

C_DEPS += \
./Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.d \
./Drivers/MRT_Helpers_f4xx/Src/w25qxx.d 

OBJS += \
./Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.o \
./Drivers/MRT_Helpers_f4xx/Src/w25qxx.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MRT_Helpers_f4xx/Src/%.o: ../Drivers/MRT_Helpers_f4xx/Src/%.c Drivers/MRT_Helpers_f4xx/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/MRT_Helpers_f4xx/Inc -I../Drivers/MRT_Iridium_f4xx/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/MRT_LSM6DSR_f4xx/Inc -I../Drivers/MRT_RTOS_f4xx/Inc -I../Drivers/MRT_GPS_f4xx/Inc -I../Drivers/MRT_LPS22HH_f4xx/Inc -I../Drivers/MRT_Thermocouple_f4xx/Inc -I../Drivers/SX1262_c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FatFs/src -I../Drivers/FATFS/Target -I../Drivers/FATFS/App -I../Drivers/SD_CARD/Inc -I../FATFS/Target -I../FATFS/App -O0 -ffunction-sections -fdata-sections -Wall -lm -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MRT_Helpers_f4xx-2f-Src

clean-Drivers-2f-MRT_Helpers_f4xx-2f-Src:
	-$(RM) ./Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.d ./Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.o ./Drivers/MRT_Helpers_f4xx/Src/w25qxx.d ./Drivers/MRT_Helpers_f4xx/Src/w25qxx.o

.PHONY: clean-Drivers-2f-MRT_Helpers_f4xx-2f-Src

