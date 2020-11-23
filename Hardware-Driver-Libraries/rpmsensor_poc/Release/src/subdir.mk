################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/rpmsensor_poc_main.cpp 

OBJS += \
./src/rpmsensor_poc_main.o 

CPP_DEPS += \
./src/rpmsensor_poc_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	aarch64-linux-gnu-g++ -I"/home/jac/JaCloud/OurCode/_03_Embedded/sandbox_embed64_ws/rpmsensor/src" -I/home/jac/JaCloud/OurCode/_03_Embedded/embed64_ws/i2c/src -include"/home/jac/JaCloud/OurCode/_03_Embedded/sandbox_embed64_ws/rpmsensor/src/rpmsensor.h" -include/home/jac/JaCloud/OurCode/_03_Embedded/embed64_ws/i2c/src/i2c.h -O3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


