################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MRT_ISM330DLC_f4xx/Src/ism330dlc_reg.c 

C_DEPS += \
./Drivers/MRT_ISM330DLC_f4xx/Src/ism330dlc_reg.d 

OBJS += \
./Drivers/MRT_ISM330DLC_f4xx/Src/ism330dlc_reg.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MRT_ISM330DLC_f4xx/Src/%.o: ../Drivers/MRT_ISM330DLC_f4xx/Src/%.c Drivers/MRT_ISM330DLC_f4xx/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Jacoby/Desktop/Engineering/MRT/Avionics/Flight Computer/avionics-2022/Flight Computer/Release/FC_v1.2/Drivers/MRT_Helpers_f4xx/Inc" -I"C:/Users/Jacoby/Desktop/Engineering/MRT/Avionics/Flight Computer/avionics-2022/Flight Computer/Release/FC_v1.2/Drivers/MRT_Iridium_f4xx/Inc" -I"C:/Users/Jacoby/Desktop/Engineering/MRT/Avionics/Flight Computer/avionics-2022/Flight Computer/Release/FC_v1.2/Drivers/MRT_ISM330DLC_f4xx/Inc" -I"C:/Users/Jacoby/Desktop/Engineering/MRT/Avionics/Flight Computer/avionics-2022/Flight Computer/Release/FC_v1.2/Drivers/MRT_RTOS_f4xx/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

