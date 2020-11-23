################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/rpmsensor.cpp 

OBJS += \
./src/rpmsensor.o 

CPP_DEPS += \
./src/rpmsensor.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	aarch64-linux-gnu-g++ -I/home/jac/JaCloud/OurCode/_03_Embedded/embed64_ws/i2c/src -include/home/jac/JaCloud/OurCode/_03_Embedded/embed64_ws/i2c/src/i2c.h -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


