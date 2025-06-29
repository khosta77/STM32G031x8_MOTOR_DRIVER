C_SRCS += \
./system/src/diag/trace-impl.c \
./system/src/diag/trace.c

C_DEPS += \
./system/src/diag/trace-impl.d \
./system/src/diag/trace.d

OBJS += \
./system/src/diag/trace-impl.o \
./system/src/diag/trace.o

system/src/diag/%.o: ./system/src/diag/%.c system/src/diag/subdir.mk
	@echo 'In process: ./system/src/diag/'
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -flto -fno-move-loop-invariants -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32G031xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"./include" -I"../system/include" -I"./system/include/cmsis" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
