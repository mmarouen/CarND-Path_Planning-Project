################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Eigen-3.3/bench/btl/libs/blitz/btl_blitz.cpp \
../src/Eigen-3.3/bench/btl/libs/blitz/btl_tiny_blitz.cpp 

OBJS += \
./src/Eigen-3.3/bench/btl/libs/blitz/btl_blitz.o \
./src/Eigen-3.3/bench/btl/libs/blitz/btl_tiny_blitz.o 

CPP_DEPS += \
./src/Eigen-3.3/bench/btl/libs/blitz/btl_blitz.d \
./src/Eigen-3.3/bench/btl/libs/blitz/btl_tiny_blitz.d 


# Each subdirectory must supply rules for building sources it contributes
src/Eigen-3.3/bench/btl/libs/blitz/%.o: ../src/Eigen-3.3/bench/btl/libs/blitz/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -D__GXX_EXPERIMENTAL_CXX0X__ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


