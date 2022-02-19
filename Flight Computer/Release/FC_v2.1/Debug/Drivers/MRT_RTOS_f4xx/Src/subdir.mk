################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MRT_RTOS_f4xx/Src/MRT_RTOS.c 

C_DEPS += \
./Drivers/MRT_RTOS_f4xx/Src/MRT_RTOS.d 

OBJS += \
./Drivers/MRT_RTOS_f4xx/Src/MRT_RTOS.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MRT_RTOS_f4xx/Src/%.o: ../Drivers/MRT_RTOS_f4xx/Src/%.c Drivers/MRT_RTOS_f4xx/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../Drivers/SX1262_c -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/MRT_Helpers_f4xx/Inc -I../Drivers/MRT_Iridium_f4xx/Inc -I../Drivers/MRT_LSM6DSR_f4xx/Inc -I../Drivers/MRT_RTOS_f4xx/Inc -I../Drivers/MRT_GPS_f4xx/Inc -I../Drivers/MRT_LPS22HH_f4xx/Inc -I../Drivers/MRT_Thermocouple_f4xx/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

