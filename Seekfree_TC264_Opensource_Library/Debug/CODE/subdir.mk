################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CODE/Binarization.c \
../CODE/BluetoothSend.c \
../CODE/FuzzyPID.c \
../CODE/ImageBasic.c \
../CODE/ImageSpecial.c \
../CODE/ImageTack.c \
../CODE/Motor.c \
../CODE/PID.c \
../CODE/Steer.c 

OBJS += \
./CODE/Binarization.o \
./CODE/BluetoothSend.o \
./CODE/FuzzyPID.o \
./CODE/ImageBasic.o \
./CODE/ImageSpecial.o \
./CODE/ImageTack.o \
./CODE/Motor.o \
./CODE/PID.o \
./CODE/Steer.o 

COMPILED_SRCS += \
./CODE/Binarization.src \
./CODE/BluetoothSend.src \
./CODE/FuzzyPID.src \
./CODE/ImageBasic.src \
./CODE/ImageSpecial.src \
./CODE/ImageTack.src \
./CODE/Motor.src \
./CODE/PID.src \
./CODE/Steer.src 

C_DEPS += \
./CODE/Binarization.d \
./CODE/BluetoothSend.d \
./CODE/FuzzyPID.d \
./CODE/ImageBasic.d \
./CODE/ImageSpecial.d \
./CODE/ImageTack.d \
./CODE/Motor.d \
./CODE/PID.d \
./CODE/Steer.d 


# Each subdirectory must supply rules for building sources it contributes
CODE/%.src: ../CODE/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/Infineon/InfinenoProject/SmartCar/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

CODE/%.o: ./CODE/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


