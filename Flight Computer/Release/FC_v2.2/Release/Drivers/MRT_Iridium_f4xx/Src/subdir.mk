################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MRT_Iridium_f4xx/Src/dtostrf.c \
../Drivers/MRT_Iridium_f4xx/Src/itoa.c 

CPP_SRCS += \
../Drivers/MRT_Iridium_f4xx/Src/IridiumSBD.cpp \
../Drivers/MRT_Iridium_f4xx/Src/IridiumSBD_Static_API.cpp \
../Drivers/MRT_Iridium_f4xx/Src/Print.cpp \
../Drivers/MRT_Iridium_f4xx/Src/Stream.cpp \
../Drivers/MRT_Iridium_f4xx/Src/WString.cpp \
../Drivers/MRT_Iridium_f4xx/Src/Wire.cpp 

C_DEPS += \
./Drivers/MRT_Iridium_f4xx/Src/dtostrf.d \
./Drivers/MRT_Iridium_f4xx/Src/itoa.d 

OBJS += \
./Drivers/MRT_Iridium_f4xx/Src/IridiumSBD.o \
./Drivers/MRT_Iridium_f4xx/Src/IridiumSBD_Static_API.o \
./Drivers/MRT_Iridium_f4xx/Src/Print.o \
./Drivers/MRT_Iridium_f4xx/Src/Stream.o \
./Drivers/MRT_Iridium_f4xx/Src/WString.o \
./Drivers/MRT_Iridium_f4xx/Src/Wire.o \
./Drivers/MRT_Iridium_f4xx/Src/dtostrf.o \
./Drivers/MRT_Iridium_f4xx/Src/itoa.o 

CPP_DEPS += \
./Drivers/MRT_Iridium_f4xx/Src/IridiumSBD.d \
./Drivers/MRT_Iridium_f4xx/Src/IridiumSBD_Static_API.d \
./Drivers/MRT_Iridium_f4xx/Src/Print.d \
./Drivers/MRT_Iridium_f4xx/Src/Stream.d \
./Drivers/MRT_Iridium_f4xx/Src/WString.d \
./Drivers/MRT_Iridium_f4xx/Src/Wire.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MRT_Iridium_f4xx/Src/%.o: ../Drivers/MRT_Iridium_f4xx/Src/%.cpp Drivers/MRT_Iridium_f4xx/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F437xx -c -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/MRT_Iridium_f4xx/Src/%.o: ../Drivers/MRT_Iridium_f4xx/Src/%.c Drivers/MRT_Iridium_f4xx/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F437xx -c -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

