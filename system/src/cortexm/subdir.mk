C_SRCS += \
./system/src/cortexm/exception-handlers.c \
./system/src/cortexm/initialize-hardware.c \
./system/src/cortexm/reset-hardware.c

C_DEPS += \
./system/src/cortexm/exception-handlers.d \
./system/src/cortexm/initialize-hardware.d \
./system/src/cortexm/reset-hardware.d

OBJS += \
./system/src/cortexm/exception-handlers.o \
./system/src/cortexm/initialize-hardware.o \
./system/src/cortexm/reset-hardware.o

system/src/cortexm/%.o: ./system/src/cortexm/%.c system/src/cortexm/subdir.mk
	@echo 'In process: ./system/src/cortexm/'
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -flto -fno-move-loop-invariants -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32G031xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"./include" -I"./system/include" -I"./system/include/cmsis" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
