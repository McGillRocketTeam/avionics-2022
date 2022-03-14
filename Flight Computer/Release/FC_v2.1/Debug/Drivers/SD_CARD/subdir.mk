################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SD_CARD/sd_card.c 

C_DEPS += \
./Drivers/SD_CARD/sd_card.d 

OBJS += \
./Drivers/SD_CARD/sd_card.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SD_CARD/%.o: ../Drivers/SD_CARD/%.c Drivers/SD_CARD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/MRT_Helpers_f4xx/Inc -I../Drivers/MRT_Iridium_f4xx/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/MRT_LSM6DSR_f4xx/Inc -I../Drivers/MRT_RTOS_f4xx/Inc -I../Drivers/MRT_GPS_f4xx/Inc -I../Drivers/MRT_LPS22HH_f4xx/Inc -I../Drivers/MRT_Thermocouple_f4xx/Inc -I../Drivers/SX1262_c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FatFs/src -I../Drivers/FATFS/Target -I../Drivers/FATFS/App -I../Drivers/SD_CARD/Inc -I../FATFS/Target -I../FATFS/App -O0 -ffunction-sections -fdata-sections -Wall -lm -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SD_CARD

clean-Drivers-2f-SD_CARD:
	-$(RM) ./Drivers/SD_CARD/sd_card.d ./Drivers/SD_CARD/sd_card.o

.PHONY: clean-Drivers-2f-SD_CARD

