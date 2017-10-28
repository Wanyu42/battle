# Beck Pang, 20171005
# cross-compiler for gcc-arm-embedded
# TARGET:  STM32F427IIHx
# https://blog.jetbrains.com/clion/2016/06/clion-for-embedded-development/


SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Mac_toolchain/STM32F427IIHx_FLASH.ld)

# Uncomment for software floating point
# Options for gcc
# -specs=nosys.specs stands for
# https://gcc.gnu.org/onlinedocs/gcc/Option-Summary.html
# -specs=nosys.specs non semi-hosting mode is not good for Ozone feedback
#SET(COMMON_FLAGS "-mcpu=cortex-m4 -mthumb -mthumb-interwork -mfloat-abi=soft -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0") -specs=nano.specs
SET(COMMON_FLAGS "  -mcpu=cortex-m4\
                    -mthumb\
                    -mthumb-interwork\
                    -mfloat-abi=hard\
                    -mfpu=fpv4-sp-d16\
                    -ffunction-sections\
                    -g\
                    -fno-common\
                    -fmessage-length=0\
                    -specs=nano.specs\
                ")
SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,-Map=binary.map -T ${LINKER_SCRIPT}")