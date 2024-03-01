################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_api.c \
../Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_calibration.c 

OBJS += \
./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_api.o \
./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_calibration.o 

C_DEPS += \
./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_api.d \
./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_calibration.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_api.o: ../Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_api.c Drivers/VL53L1X\ Drivers/API/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"E:/Projects/Electronics/Personal Projects/Overhead Tank/STM32 Practise/Sensor Reading/Drivers/VL53L1X Drivers/API/platform/inc" -I"E:/Projects/Electronics/Personal Projects/Overhead Tank/STM32 Practise/Sensor Reading/Drivers/VL53L1X Drivers/API/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/VL53L1X Drivers/API/core/src/VL53L1X_api.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_calibration.o: ../Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_calibration.c Drivers/VL53L1X\ Drivers/API/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"E:/Projects/Electronics/Personal Projects/Overhead Tank/STM32 Practise/Sensor Reading/Drivers/VL53L1X Drivers/API/platform/inc" -I"E:/Projects/Electronics/Personal Projects/Overhead Tank/STM32 Practise/Sensor Reading/Drivers/VL53L1X Drivers/API/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/VL53L1X Drivers/API/core/src/VL53L1X_calibration.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L1X-20-Drivers-2f-API-2f-core-2f-src

clean-Drivers-2f-VL53L1X-20-Drivers-2f-API-2f-core-2f-src:
	-$(RM) ./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_api.cyclo ./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_api.d ./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_api.o ./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_api.su ./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_calibration.cyclo ./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_calibration.d ./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_calibration.o ./Drivers/VL53L1X\ Drivers/API/core/src/VL53L1X_calibration.su

.PHONY: clean-Drivers-2f-VL53L1X-20-Drivers-2f-API-2f-core-2f-src

