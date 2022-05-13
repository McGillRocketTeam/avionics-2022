################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MRT_MISC/Src/video_recorder.c 

C_DEPS += \
./Drivers/MRT_MISC/Src/video_recorder.d 

OBJS += \
./Drivers/MRT_MISC/Src/video_recorder.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MRT_MISC/Src/%.o: ../Drivers/MRT_MISC/Src/%.c Drivers/MRT_MISC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/MRT_EJECTION/Inc -I../Drivers/MRT_MEMORY/Inc -I../Drivers/MRT_PROPULSION/Inc -I../Drivers/MRT_SENSORS/Inc -I../Drivers/MRT_TELEMETRY/Inc -I../Drivers/MRT_WATCHDOG/Inc -I../Drivers/MRT_MISC/Inc -I../Drivers/MRT_TELEMETRY/Inc/MRT_Iridium_f4xx/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MRT_MISC-2f-Src

clean-Drivers-2f-MRT_MISC-2f-Src:
	-$(RM) ./Drivers/MRT_MISC/Src/video_recorder.d ./Drivers/MRT_MISC/Src/video_recorder.o

.PHONY: clean-Drivers-2f-MRT_MISC-2f-Src

