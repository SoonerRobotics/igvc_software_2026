################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/Hardware/CanSparkMax.cpp 

OBJS += \
./Core/Inc/Hardware/CanSparkMax.o 

CPP_DEPS += \
./Core/Inc/Hardware/CanSparkMax.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Hardware/%.o Core/Inc/Hardware/%.su Core/Inc/Hardware/%.cyclo: ../Core/Inc/Hardware/%.cpp Core/Inc/Hardware/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F105xC -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Hardware

clean-Core-2f-Inc-2f-Hardware:
	-$(RM) ./Core/Inc/Hardware/CanSparkMax.cyclo ./Core/Inc/Hardware/CanSparkMax.d ./Core/Inc/Hardware/CanSparkMax.o ./Core/Inc/Hardware/CanSparkMax.su

.PHONY: clean-Core-2f-Inc-2f-Hardware

