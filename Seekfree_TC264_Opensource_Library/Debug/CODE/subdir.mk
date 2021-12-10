################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CODE/Binarization.c \
../CODE/ImageBasic.c \
../CODE/PID.c \
../CODE/Steer.c 

OBJS += \
./CODE/Binarization.o \
./CODE/ImageBasic.o \
./CODE/PID.o \
./CODE/Steer.o 

COMPILED_SRCS += \
./CODE/Binarization.src \
./CODE/ImageBasic.src \
./CODE/PID.src \
./CODE/Steer.src 

C_DEPS += \
./CODE/Binarization.d \
./CODE/ImageBasic.d \
./CODE/PID.d \
./CODE/Steer.d 


# Each subdirectory must supply rules for building sources it contributes
CODE/%.src: ../CODE/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fE:/nodeanddata/studio/FSL/Complete/S17/SmartCar/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

CODE/%.o: ./CODE/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


