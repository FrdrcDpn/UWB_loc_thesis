cmd_src/modules/src/attitude_pid_controller.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/.attitude_pid_controller.o.d    -I../src/modules/src -Isrc/modules/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW  -I../../vendor/CMSIS/CMSIS/Core/Include -I../vendor/CMSIS/CMSIS/Core/Include  -I../../vendor/CMSIS/CMSIS/DSP/Include -I../vendor/CMSIS/CMSIS/DSP/Include  -I../../vendor/libdw1000/inc -I../vendor/libdw1000/inc  -I../../vendor/FreeRTOS/include -I../vendor/FreeRTOS/include  -I../../vendor/FreeRTOS/portable/GCC/ARM_CM4F -I../vendor/FreeRTOS/portable/GCC/ARM_CM4F  -I../../src/config -I../src/config  -I../../src/platform/interface -I../src/platform/interface  -I../../src/deck/interface -I../src/deck/interface  -I../../src/deck/drivers/interface -I../src/deck/drivers/interface  -I../../src/drivers/interface -I../src/drivers/interface  -I../../src/drivers/bosch/interface -I../src/drivers/bosch/interface  -I../../src/drivers/esp32/interface -I../src/drivers/esp32/interface  -I../../src/hal/interface -I../src/hal/interface  -I../../src/modules/interface -I../src/modules/interface  -I../../src/modules/interface/kalman_core -I../src/modules/interface/kalman_core  -I../../src/modules/interface/lighthouse -I../src/modules/interface/lighthouse  -I../../src/utils/interface -I../src/utils/interface  -I../../src/utils/interface/kve -I../src/utils/interface/kve  -I../../src/utils/interface/lighthouse -I../src/utils/interface/lighthouse  -I../../src/utils/interface/tdoa -I../src/utils/interface/tdoa  -I../../src/lib/FatFS -I../src/lib/FatFS  -I../../src/lib/CMSIS/STM32F4xx/Include -I../src/lib/CMSIS/STM32F4xx/Include  -I../../src/lib/STM32_USB_Device_Library/Core/inc -I../src/lib/STM32_USB_Device_Library/Core/inc  -I../../src/lib/STM32_USB_OTG_Driver/inc -I../src/lib/STM32_USB_OTG_Driver/inc  -I../../src/lib/STM32F4xx_StdPeriph_Driver/inc -I../src/lib/STM32F4xx_StdPeriph_Driver/inc  -I../../src/lib/vl53l1 -I../src/lib/vl53l1  -I../../src/lib/vl53l1/core/inc -I../src/lib/vl53l1/core/inc  -I../build/include/generated -Ibuild/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -Wno-address-of-packed-member -DLOCODECK_USE_DOUBLE_DECK -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o src/modules/src/attitude_pid_controller.o ../src/modules/src/attitude_pid_controller.c

source_src/modules/src/attitude_pid_controller.o := ../src/modules/src/attitude_pid_controller.c

deps_src/modules/src/attitude_pid_controller.o := \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  ../src/modules/interface/stabilizer_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
  ../src/hal/interface/imu_types.h \
  ../src/utils/interface/lighthouse/lighthouse_types.h \
  ../src/modules/interface/attitude_controller.h \
  ../src/modules/interface/pid.h \
  ../src/utils/interface/filter.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/math.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/reent.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/newlib.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/ieeefp.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/lock.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/cdefs.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  ../src/modules/interface/param.h \
  ../src/modules/interface/param_logic.h \
  ../src/modules/interface/crtp.h \
  ../src/modules/interface/log.h \

src/modules/src/attitude_pid_controller.o: $(deps_src/modules/src/attitude_pid_controller.o)

$(deps_src/modules/src/attitude_pid_controller.o):
