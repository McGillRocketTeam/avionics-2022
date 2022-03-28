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
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F437xx -c -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

