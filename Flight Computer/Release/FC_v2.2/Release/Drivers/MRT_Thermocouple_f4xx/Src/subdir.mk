################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MRT_Thermocouple_f4xx/Src/MAX31855.c 

C_DEPS += \
./Drivers/MRT_Thermocouple_f4xx/Src/MAX31855.d 

OBJS += \
./Drivers/MRT_Thermocouple_f4xx/Src/MAX31855.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MRT_Thermocouple_f4xx/Src/%.o: ../Drivers/MRT_Thermocouple_f4xx/Src/%.c Drivers/MRT_Thermocouple_f4xx/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F437xx -c -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

