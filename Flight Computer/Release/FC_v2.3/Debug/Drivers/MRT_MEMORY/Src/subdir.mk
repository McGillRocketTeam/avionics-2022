################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MRT_MEMORY/Src/MRT_external_flash.c \
../Drivers/MRT_MEMORY/Src/w25qxx.c 

C_DEPS += \
./Drivers/MRT_MEMORY/Src/MRT_external_flash.d \
./Drivers/MRT_MEMORY/Src/w25qxx.d 

OBJS += \
./Drivers/MRT_MEMORY/Src/MRT_external_flash.o \
./Drivers/MRT_MEMORY/Src/w25qxx.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MRT_MEMORY/Src/%.o: ../Drivers/MRT_MEMORY/Src/%.c Drivers/MRT_MEMORY/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/MRT_EJECTION/Inc -I../Drivers/MRT_MEMORY/Inc -I../Drivers/MRT_PROPULSION/Inc -I../Drivers/MRT_SENSORS/Inc -I../Drivers/MRT_TELEMETRY/Inc -I../Drivers/MRT_WATCHDOG/Inc -I../Drivers/MRT_MISC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

