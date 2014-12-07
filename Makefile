# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
# 


OPT = 0

# Object files directory
# Warning: this will be removed by make clean!
#
OBJDIR = obj

# Target file name (without extension)
TARGET = $(OBJDIR)/STM32F4_Test

# Define all C source files (dependencies are generated automatically)
#

#---------- added Matic Knap 26 Jun 2014 -----------------

SOURCES += App/STM32F4-Discovery/spi.c
SOURCES += App/STM32F4-Discovery/W5200.c
SOURCES += App/STM32F4-Discovery/modbus_mk.c
SOURCES += App/STM32F4-Discovery/tcpCLI.c

#SOURCES += App/STM32F4-Discovery/printf_my.c
SOURCES += App/STM32F4-Discovery/printf-stdarg.c


#---------- added Matic Knap 26 Jun 2014 -----------------

SOURCES += App/STM32F4-Discovery/main.c
SOURCES += App/STM32F4-Discovery/startup/startup_stm32f4xx.s
SOURCES += App/STM32F4-Discovery/startup/system_stm32f4xx.c

#free-rtos source code
SOURCES += Source/tasks.c
SOURCES += Source/queue.c
SOURCES += Source/list.c
SOURCES += Source/croutine.c
SOURCES += Source/portable/GCC/ARM_CM4F/port.c 
SOURCES += Source/timers.c
SOURCES += Source/printf.c

# free-rtos plus source code 
SOURCES += Source/FreeRTOS-Plus-CLI/FreeRTOS_CLI.c

#Memory management
SOURCES += Source/portable/MemMang/heap_4.c
#free-rtos common source code
#SOURCES += App/Common/Full/BlockQ.c
#SOURCES += App/Common/Full/comtest.c
#SOURCES += App/Common/Full/death.c
#SOURCES += App/Common/Full/dynamic.c
#SOURCES += App/Common/Full/events.c
#SOURCES += App/Common/Full/flash.c
#SOURCES += App/Common/Full/flop.c
#SOURCES += App/Common/Full/integer.c
#SOURCES += App/Common/Full/PollQ.c
#SOURCES += App/Common/Full/print.c
#SOURCES += App/Common/Full/semtest.c

#SOURCES += App/Common/Minimal/AltBlckQ.c
#SOURCES += App/Common/Minimal/AltBlock.c
#SOURCES += App/Common/Minimal/AltPollQ.c
#SOURCES += App/Common/Minimal/AltQTest.c
SOURCES += App/Common/Minimal/BlockQ.c
SOURCES += App/Common/Minimal/blocktim.c
#SOURCES += App/Common/Minimal/comtest.c
#SOURCES += App/Common/Minimal/comtest_strings.c
SOURCES += App/Common/Minimal/countsem.c
#SOURCES += App/Common/Minimal/crflash.c
#SOURCES += App/Common/Minimal/crhook.c
SOURCES += App/Common/Minimal/death.c
SOURCES += App/Common/Minimal/dynamic.c
SOURCES += App/Common/Minimal/flash.c
#SOURCES += App/Common/Minimal/flash_timer.c
SOURCES += App/Common/Minimal/flop.c
SOURCES += App/Common/Minimal/GenQTest.c
SOURCES += App/Common/Minimal/integer.c
#SOURCES += App/Common/Minimal/IntQueue.c
SOURCES += App/Common/Minimal/PollQ.c
#SOURCES += App/Common/Minimal/QPeek.c
#SOURCES += App/Common/Minimal/QueueOverwrite.c
#SOURCES += App/Common/Minimal/QueueSet.c
SOURCES += App/Common/Minimal/recmutex.c
SOURCES += App/Common/Minimal/semtest.c
#SOURCES += App/Common/Minimal/sp_flop.c
#SOURCES += App/Common/Minimal/TimerDemo.c

#Board Init
SOURCES += App/STM32F4-Discovery/board/stm32f4_discovery.c
#STM32F4xx library 
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
SOURCES += App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c

OBJECTS  = $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(SOURCES))))

# Place -D, -U or -I options here for C and C++ sources
CPPFLAGS += -ISource
CPPFLAGS += -ISource/include/
CPPFLAGS += -Ibojan/
CPPFLAGS += -ISource/FreeRTOS-Plus-CLI/
CPPFLAGS += -IApp/STM32F4-Discovery/Libraries/CMSIS/Include
CPPFLAGS += -IApp/STM32F4-Discovery/Libraries/CMSIS/ST/STM32F4xx/Include
CPPFLAGS += -IApp/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/inc
CPPFLAGS += -IApp/STM32F4-Discovery
CPPFLAGS += -ISource/portable/GCC/ARM_CM4F
CPPFLAGS += -IApp/Common/include
CPPFLAGS += -IApp/STM32F4-Discovery/board
#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS  = -O$(OPT)
#CFLAGS += -std=gnu99
CFLAGS += -gdwarf-2
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
#CFLAGS += -Wall
CFLAGS += -w
#CFLAGS += -Wextra
#CFLAGS += -Wpointer-arith
#CFLAGS += -Wstrict-prototypes
#CFLAGS += -Winline
#CFLAGS += -Wunreachable-code
#CFLAGS += -Wundef
CFLAGS += -Wa,-adhlns=$(OBJDIR)/$(*F).lst
CFLAGS += -DITM_TRACE
CFLAGS+=-DCORTEX_M4
#CFLAGS+=-DCORTEX_M3
CFLAGS+=-DSTM32F4xx

# Optimize use of the single-precision FPU
#
CFLAGS += -fsingle-precision-constant

# use ST peripheral driver
CFLAGS += -DUSE_STDPERIPH_DRIVER
# use USE_FULL_ASSERT
CFLAGS += -DUSE_FULL_ASSERT
# This will not work without recompiling App/STM32F4-Discovery/Libraries
#
# CFLAGS += -fshort-double

#---------------- Compiler Options C++ ----------------
#
CXXFLAGS  = $(CFLAGS)

#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler
#  -adhlns:   create listing
#
ASFLAGS = -Wa,-adhlns=$(OBJDIR)/$(*F).lst


#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS += -lm
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -TApp/STM32F4-Discovery/config/stm32_flash.ld

#============================================================================


# Define programs and commands
TOOLCHAIN = arm-none-eabi
CC        = $(TOOLCHAIN)-gcc
OBJCOPY   = $(TOOLCHAIN)-objcopy
OBJDUMP   = $(TOOLCHAIN)-objdump
SIZE      = $(TOOLCHAIN)-size
NM        = $(TOOLCHAIN)-nm
OPENOCD   = openocd
DOXYGEN   = doxygen
STLINK    = st-flash


ifeq (AMD64, $(PROCESSOR_ARCHITEW6432))
  SUBWCREV = tools/SubWCRev64.exe
else
  SUBWCREV = tools/SubWCRev.exe
endif


# Compiler flags to generate dependency files
GENDEPFLAGS = -MMD -MP -MF $(OBJDIR)/$(*F).d


# Combine all necessary flags and optional flags
# Add target processor to flags.
#
#CPU = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
#CPU = -mcpu=cortex-m4 -mthumb 
#

CPU = -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16

CFLAGS   += $(CPU)
CXXFLAGS += $(CPU)
ASFLAGS  += $(CPU)
LDFLAGS  += $(CPU)

# Default target.
all:  gccversion build showsize

build: elf hex lss sym bin

elf: $(TARGET).elf
hex: $(TARGET).hex
bin: $(TARGET).bin
lss: $(TARGET).lss
sym: $(TARGET).sym


doxygen:
	@echo
	@echo Creating Doxygen documentation
	@$(DOXYGEN)

# Display compiler version information
gccversion: 
	@$(CC) --version


# Show the final program size
showsize: elf
	@echo
	@$(SIZE) $(TARGET).elf 2>/dev/null


# Flash the device  
flash: hex
#	$(OPENOCD) -f "openocd.cfg" -c "flash_image $(TARGET).elf; shutdown"
	$(STLINK) write $(TARGET).bin 0x08000000


# Target: clean project
clean:
	@echo Cleaning project:
	rm -rf $(OBJDIR)
	rm -rf docs/html


# Create extended listing file from ELF output file
%.lss: %.elf
	@echo
	@echo Creating Extended Listing: $@
	$(OBJDUMP) -h -S -z $< > $@


# Create a symbol table from ELF output file
%.sym: %.elf
	@echo
	@echo Creating Symbol Table: $@
	$(NM) -n $< > $@


# Link: create ELF output file from object files
.SECONDARY: $(TARGET).elf
.PRECIOUS:  $(OBJECTS)
$(TARGET).elf: $(OBJECTS)
	@echo
	@echo Linking: $@
	$(CC) $^ $(LDFLAGS) --output $@ 


# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo Creating hex file: $@
	$(OBJCOPY) -O ihex $< $@

# Create bin file :
%.bin: %.elf
	@echo
	@echo Creating bin file: $@
	$(OBJCOPY) -O binary $< $@


# Compile: create object files from C source files
$(OBJDIR)/%.o : %.c
	@echo
	@echo Compiling C: $<
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $(GENDEPFLAGS) $< -o $@ 


# Compile: create object files from C++ source files
$(OBJDIR)/%.o : %.cpp
	@echo
	@echo Compiling CPP: $<
	$(CC) -c $(CPPFLAGS) $(CXXFLAGS) $(GENDEPFLAGS) $< -o $@ 


# Assemble: create object files from assembler source files
$(OBJDIR)/%.o : %.s
	@echo
	@echo Assembling: $<
	$(CC) -c $(CPPFLAGS) $(ASFLAGS) $< -o $@


# Create object file directories
$(shell mkdir -p $(OBJDIR) 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/App/STM32F4-Discovery 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/Source/ 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/Source/FreeRTOS-Plus-CLI 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/Source/portable/GCC/ARM_CM4F 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/App/STM32F4-Discovery/Libraries/STM32F4xx_StdPeriph_Driver/src 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/App/STM32F4-Discovery/startup 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/App/Common/Full 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/App/Common/Minimal 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/App/STM32F4-Discovery/board 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/Source/portable/MemMang 2>/dev/null)

# Include the dependency files
-include $(wildcard $(OBJDIR)/*.d)


# Listing of phony targets
.PHONY: all build flash clean \
        doxygen elf lss sym \
        showsize gccversion
