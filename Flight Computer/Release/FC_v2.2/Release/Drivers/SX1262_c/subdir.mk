################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SX1262_c/sx126x.c 

C_DEPS += \
./Drivers/SX1262_c/sx126x.d 

OBJS += \
./Drivers/SX1262_c/sx126x.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SX1262_c/%.o: ../Drivers/SX1262_c/%.c Drivers/SX1262_c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F437xx -c -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

