################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L1X\ Drivers/API/platform/src/vl53l1_platform.c 

OBJS += \
./Drivers/VL53L1X\ Drivers/API/platform/src/vl53l1_platform.o 

C_DEPS += \
./Drivers/VL53L1X\ Drivers/API/platform/src/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L1X\ Drivers/API/platform/src/vl53l1_platform.o: ../Drivers/VL53L1X\ Drivers/API/platform/src/vl53l1_platform.c Drivers/VL53L1X\ Drivers/API/platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"E:/Projects/Electronics/Personal Projects/Overhead Tank/STM32 Practise/Sensor Reading/Drivers/VL53L1X Drivers/API/platform/inc" -I"E:/Projects/Electronics/Personal Projects/Overhead Tank/STM32 Practise/Sensor Reading/Drivers/VL53L1X Drivers/API/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/VL53L1X Drivers/API/platform/src/vl53l1_platform.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L1X-20-Drivers-2f-API-2f-platform-2f-src

clean-Drivers-2f-VL53L1X-20-Drivers-2f-API-2f-platform-2f-src:
	-$(RM) ./Drivers/VL53L1X\ Drivers/API/platform/src/vl53l1_platform.cyclo ./Drivers/VL53L1X\ Drivers/API/platform/src/vl53l1_platform.d ./Drivers/VL53L1X\ Drivers/API/platform/src/vl53l1_platform.o ./Drivers/VL53L1X\ Drivers/API/platform/src/vl53l1_platform.su

.PHONY: clean-Drivers-2f-VL53L1X-20-Drivers-2f-API-2f-platform-2f-src

