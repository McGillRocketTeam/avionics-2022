################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/dtostrf.c \
../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/itoa.c 

CPP_SRCS += \
../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/IridiumSBD.cpp \
../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/MRT_iridium.cpp \
../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Print.cpp \
../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Stream.cpp \
../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/WString.cpp \
../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Wire.cpp 

C_DEPS += \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/dtostrf.d \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/itoa.d 

OBJS += \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/IridiumSBD.o \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/MRT_iridium.o \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Print.o \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Stream.o \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/WString.o \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Wire.o \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/dtostrf.o \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/itoa.o 

CPP_DEPS += \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/IridiumSBD.d \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/MRT_iridium.d \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Print.d \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Stream.d \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/WString.d \
./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Wire.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/%.o: ../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/%.cpp Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/MRT_EJECTION/Inc -I../Drivers/MRT_MEMORY/Inc -I../Drivers/MRT_PROPULSION/Inc -I../Drivers/MRT_SENSORS/Inc -I../Drivers/MRT_TELEMETRY/Inc -I../Drivers/MRT_WATCHDOG/Inc -I../Drivers/MRT_MISC/Inc -I../Drivers/MRT_TELEMETRY/Inc/MRT_Iridium_f4xx/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/%.o: ../Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/%.c Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F437xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/MRT_EJECTION/Inc -I../Drivers/MRT_MEMORY/Inc -I../Drivers/MRT_PROPULSION/Inc -I../Drivers/MRT_SENSORS/Inc -I../Drivers/MRT_TELEMETRY/Inc -I../Drivers/MRT_WATCHDOG/Inc -I../Drivers/MRT_MISC/Inc -I../Drivers/MRT_TELEMETRY/Inc/MRT_Iridium_f4xx/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MRT_TELEMETRY-2f-Src-2f-MRT_Iridium_f4xx-2f-Src

clean-Drivers-2f-MRT_TELEMETRY-2f-Src-2f-MRT_Iridium_f4xx-2f-Src:
	-$(RM) ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/IridiumSBD.d ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/IridiumSBD.o ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/MRT_iridium.d ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/MRT_iridium.o ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Print.d ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Print.o ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Stream.d ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Stream.o ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/WString.d ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/WString.o ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Wire.d ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/Wire.o ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/dtostrf.d ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/dtostrf.o ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/itoa.d ./Drivers/MRT_TELEMETRY/Src/MRT_Iridium_f4xx/Src/itoa.o

.PHONY: clean-Drivers-2f-MRT_TELEMETRY-2f-Src-2f-MRT_Iridium_f4xx-2f-Src

