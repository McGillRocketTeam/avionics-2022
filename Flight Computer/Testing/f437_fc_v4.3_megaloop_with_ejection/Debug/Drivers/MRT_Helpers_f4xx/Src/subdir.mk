################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.c 

OBJS += \
./Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.o 

C_DEPS += \
./Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MRT_Helpers_f4xx/Src/%.o: ../Drivers/MRT_Helpers_f4xx/Src/%.c Drivers/MRT_Helpers_f4xx/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../Drivers/MRT_Helpers_f4xx/Inc -I../Drivers/MRT_RTOS_f4xx/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MRT_Helpers_f4xx-2f-Src

clean-Drivers-2f-MRT_Helpers_f4xx-2f-Src:
	-$(RM) ./Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.d ./Drivers/MRT_Helpers_f4xx/Src/MRT_Helpers.o

.PHONY: clean-Drivers-2f-MRT_Helpers_f4xx-2f-Src

