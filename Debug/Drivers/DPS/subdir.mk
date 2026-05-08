################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DPS/dps310.c 

OBJS += \
./Drivers/DPS/dps310.o 

C_DEPS += \
./Drivers/DPS/dps310.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DPS/%.o Drivers/DPS/%.su Drivers/DPS/%.cyclo: ../Drivers/DPS/%.c Drivers/DPS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/Users/YASH TATODE/STM32CubeIDE/workspace_1.18.1/data/Drivers/DPS" -I"C:/Users/YASH TATODE/STM32CubeIDE/workspace_1.18.1/data/Drivers/ICM20948" -I"C:/Users/YASH TATODE/STM32CubeIDE/workspace_1.18.1/data/Drivers/ADXL" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-DPS

clean-Drivers-2f-DPS:
	-$(RM) ./Drivers/DPS/dps310.cyclo ./Drivers/DPS/dps310.d ./Drivers/DPS/dps310.o ./Drivers/DPS/dps310.su

.PHONY: clean-Drivers-2f-DPS

