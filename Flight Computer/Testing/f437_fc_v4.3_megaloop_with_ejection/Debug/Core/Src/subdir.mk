################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MAX31855.c \
../Core/Src/adc.c \
../Core/Src/dma.c \
../Core/Src/ejection.c \
../Core/Src/gpio.c \
../Core/Src/gps.c \
../Core/Src/i2c.c \
../Core/Src/i2c_sensors_functions.c \
../Core/Src/lps22hh_reg.c \
../Core/Src/lsm6dsl_reg.c \
../Core/Src/main.c \
../Core/Src/radio_commands.c \
../Core/Src/rtc.c \
../Core/Src/sd_card.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/sx126x.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/video_recorder.c \
../Core/Src/w25qxx.c 

OBJS += \
./Core/Src/MAX31855.o \
./Core/Src/adc.o \
./Core/Src/dma.o \
./Core/Src/ejection.o \
./Core/Src/gpio.o \
./Core/Src/gps.o \
./Core/Src/i2c.o \
./Core/Src/i2c_sensors_functions.o \
./Core/Src/lps22hh_reg.o \
./Core/Src/lsm6dsl_reg.o \
./Core/Src/main.o \
./Core/Src/radio_commands.o \
./Core/Src/rtc.o \
./Core/Src/sd_card.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/sx126x.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/video_recorder.o \
./Core/Src/w25qxx.o 

C_DEPS += \
./Core/Src/MAX31855.d \
./Core/Src/adc.d \
./Core/Src/dma.d \
./Core/Src/ejection.d \
./Core/Src/gpio.d \
./Core/Src/gps.d \
./Core/Src/i2c.d \
./Core/Src/i2c_sensors_functions.d \
./Core/Src/lps22hh_reg.d \
./Core/Src/lsm6dsl_reg.d \
./Core/Src/main.d \
./Core/Src/radio_commands.d \
./Core/Src/rtc.d \
./Core/Src/sd_card.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/sx126x.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/video_recorder.d \
./Core/Src/w25qxx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
<<<<<<< HEAD:Flight Computer/Testing/f437_fc_v4.3_megaloop_with_ejection/Debug/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MAX31855.d ./Core/Src/MAX31855.o ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/ejection.d ./Core/Src/ejection.o ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c_sensors_functions.d ./Core/Src/i2c_sensors_functions.o ./Core/Src/lps22hh_reg.d ./Core/Src/lps22hh_reg.o ./Core/Src/lsm6dsl_reg.d ./Core/Src/lsm6dsl_reg.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/radio_commands.d ./Core/Src/radio_commands.o ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/sd_card.d ./Core/Src/sd_card.o ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/sx126x.d ./Core/Src/sx126x.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/video_recorder.d ./Core/Src/video_recorder.o ./Core/Src/w25qxx.d ./Core/Src/w25qxx.o

.PHONY: clean-Core-2f-Src
=======
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../Drivers/MRT_Helpers_f4xx/Inc -I../Drivers/MRT_RTOS_f4xx/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
>>>>>>> 8ac917e967fff8dacebb92ea5c1e90709dd105f7:Flight Computer/Testing/f437_fc_v4.3_megaloop/Debug/Core/Src/subdir.mk

