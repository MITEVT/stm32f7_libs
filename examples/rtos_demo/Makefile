#=============================================================================#
# ARM makefile
#
# author: Freddie Chopin, http://www.freddiechopin.info/
# last change: 2012-01-08
#
# this makefile is based strongly on many examples found in the network
#=============================================================================#

#=============================================================================#
# toolchain configuration
#=============================================================================#

TOOLCHAIN = arm-none-eabi-

CC = $(TOOLCHAIN)gcc
AS = $(TOOLCHAIN)gcc -x assembler-with-cpp
OBJCOPY = $(TOOLCHAIN)objcopy
OBJDUMP = $(TOOLCHAIN)objdump
SIZE = $(TOOLCHAIN)size
GDB = $(TOOLCHAIN)gdb
RM = rm -f

OCD_DIR = /usr/local/share/openocd/scripts
OCD = openocd

#=============================================================================#
# project configuration
#=============================================================================#

# project name
PROJECT = rtos_demo

# core type
CORE = cortex-m7

# device family
FAMILY = STM32F746xx

CUBE_DIR = ../../stm32f7_cube

# linker script
LD_SCRIPT = linker/STM32F746ZGTx_FLASH.ld

# output folder (absolute or relative path, leave empty for in-tree compilation)
OUT_DIR = bin

# C definitions
C_DEFS = -D$(FAMILY) -DUSE_FULL_LL_DRIVER

# ASM definitions
AS_DEFS = -D__STARTUP_CLEAR_BSS

# include directories (absolute or relative paths to additional folders with
# headers, current folder is always included)
INC_RTOS_DIRS = $(CUBE_DIR)/FreeRTOS/Source/include $(CUBE_DIR)/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 $(CUBE_DIR)/FreeRTOS-Plus-CLI
INC_DIRS = inc $(CUBE_DIR)/cmsis_inc $(CUBE_DIR)/cmsis_dev_inc $(CUBE_DIR)/hal_inc $(INC_RTOS_DIRS)

# library directories (absolute or relative paths to additional folders with
# libraries)
LIB_DIRS = $(CUBE_DIR)/cmsis_lib/

# libraries (additional libraries for linking, e.g. "-lm -lsome_name" to link
# math library libm.a and libsome_name.a)
LIBS =

# additional directories with source files (absolute or relative paths to
# folders with source files, current folder is always included)
SRCS_DIRS = src $(CUBE_DIR)/hal_src

# extension of C files
C_EXT = c

RTOS_SRCS = tasks.c queue.c list.c portable/GCC/ARM_CM7/r0p1/port.c portable/MemMang/heap_4.c
RTOS_PLUS_SRCS = $(CUBE_DIR)/FreeRTOS-Plus-CLI/FreeRTOS_CLI.c

# wildcard for C source files (all files with C_EXT extension found in current
# folder and SRCS_DIRS folders will be compiled and linked)
C_SRCS1 = $(wildcard $(patsubst %, %/*.$(C_EXT), . $(SRCS_DIRS)))  $(addprefix $(CUBE_DIR)/FreeRTOS/Source/, $(RTOS_SRCS)) $(RTOS_PLUS_SRCS)
FILTER_OUT = $(foreach v,$(2),$(if $(findstring $(1),$(v)),,$(v)))
C_SRCS = $(call FILTER_OUT,template, $(C_SRCS1))

# extension of ASM files
AS_EXT = s

# wildcard for ASM source files (all files with AS_EXT extension found in
# current folder and SRCS_DIRS folders will be compiled and linked)
AS_SRCS = $(wildcard $(patsubst %, %/*.$(AS_EXT), . $(SRCS_DIRS))) 


# optimization flags ("-O0" - no optimization, "-O1" - optimize, "-O2" -
# optimize even more, "-Os" - optimize for size or "-O3" - optimize yet more) 
OPTIMIZATION = -O0

# set to 1 to optimize size by removing unused code and data during link phase
REMOVE_UNUSED = 1

# define warning options here
C_WARNINGS = -Wall -Wstrict-prototypes -Wextra -fdiagnostics-color=always

# C language standard ("c89" / "iso9899:1990", "iso9899:199409",
# "c99" / "iso9899:1999", "gnu89" - default, "gnu99")
C_STD = c99

#=============================================================================#
# Write Configuration
#=============================================================================#

MOUNT = /Volumes/NODE_F746ZG/
UART_BAUD = 115200
UART_COM = $(word 1, $(wildcard /dev/tty.usbserial-*) $(wildcard /dev/tty.usbmodem*) $(wildcard /dev/ttyUSB*) $(wildcard /dev/ttyACM*))

#=============================================================================#
# set the VPATH according to SRCS_DIRS and startup file directory
#=============================================================================#

VPATH = $(SRCS_DIRS) $(CUBE_DIR)/cmsis_dev_startup $(CUBE_DIR)/hal_src $(CUBE_DIR)/FreeRTOS/Source $(CUBE_DIR)/FreeRTOS/Source/portable/MemMang $(CUBE_DIR)/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 $(CUBE_DIR)/FreeRTOS-Plus-CLI

#=============================================================================#
# when using output folder, append trailing slash to its name
#=============================================================================#

ifeq ($(strip $(OUT_DIR)), )
	OUT_DIR_F =
else
	OUT_DIR_F = $(strip $(OUT_DIR))/
endif

#=============================================================================#
# when using output folder, append trailing slash to its name
#=============================================================================#

ifeq ($(strip $(OUT_DIR_TEST)), )
	OUT_DIR_TEST_F =
else
	OUT_DIR_TEST_F = $(strip $(OUT_DIR_TEST))/
endif

#=============================================================================#
# various compilation/programming flags
#=============================================================================#

# core flags
CORE_FLAGS = -mcpu=$(CORE) -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard

# flags for C compiler
C_FLAGS = -std=$(C_STD) -g -ggdb3 -fverbose-asm -Wa,-ahlms=$(OUT_DIR_F)$(notdir $(<:.$(C_EXT)=.lst))
#			c standard	debug(?) 	extra comments	

# flags for assembler
AS_FLAGS = -g -ggdb3 -Wa,-amhls=$(OUT_DIR_F)$(notdir $(<:.$(AS_EXT)=.lst))

# flags for linker
LD_FLAGS = -T$(LD_SCRIPT) -g -Wl,-Map=$(OUT_DIR_F)$(PROJECT).map,--cref

OCD_FLAGS = -f openocd.cfg

# process option for removing unused code
ifeq ($(REMOVE_UNUSED), 1)
	# enable garbage collection of unused sections
	LD_FLAGS += -Wl,--gc-sections
	# put functions and data into their own sections
	OPTIMIZATION += -ffunction-sections -fdata-sections
endif

#=============================================================================#
# do some formatting
#=============================================================================#

FAMILY_LC = $(shell echo $(FAMILY) | tr 'A-Z' 'a-z')

AS_SRCS_F = $(AS_SRCS) $(CUBE_DIR)/cmsis_dev_startup/startup_$(FAMILY_LC).s

C_OBJS = $(addprefix $(OUT_DIR_F), $(notdir $(C_SRCS:.$(C_EXT)=.o)))
AS_OBJS = $(addprefix $(OUT_DIR_F), $(notdir $(AS_SRCS_F:.$(AS_EXT)=.o)))
OBJS = $(AS_OBJS) $(C_OBJS) $(USER_OBJS)
DEPS = $(OBJS:.o=.d)
INC_DIRS_F = -I. $(patsubst %, -I%, $(INC_DIRS))
LIB_DIRS_F = $(patsubst %, -L%, $(LIB_DIRS))

ELF = $(OUT_DIR_F)$(PROJECT).elf
HEX = $(OUT_DIR_F)$(PROJECT).hex
BIN = $(OUT_DIR_F)$(PROJECT).bin
LSS = $(OUT_DIR_F)$(PROJECT).lss
DMP = $(OUT_DIR_F)$(PROJECT).dmp
DIS = $(OUT_DIR_F)$(PROJECT).dis

C_DEFS_F = $(C_DEFS) -DUART_BAUD=$(UART_BAUD)

# format final flags for tools, request dependancies for C and asm
C_FLAGS_F = $(CORE_FLAGS) $(OPTIMIZATION) $(C_WARNINGS) $(C_FLAGS) $(C_DEFS_F) -MD -MP -MF $(OUT_DIR_F)$(@F:.o=.d) $(INC_DIRS_F)
AS_FLAGS_F = $(CORE_FLAGS) $(AS_FLAGS) $(AS_DEFS) -MD -MP -MF $(OUT_DIR_F)$(@F:.o=.d) $(INC_DIRS_F)
LD_FLAGS_F = $(CORE_FLAGS) $(LD_FLAGS) $(LIB_DIRS_F)

#contents of output directory
GENERATED = $(wildcard $(patsubst %, $(OUT_DIR_F)*.%, bin d dmp elf hex lss lst map o dis))

#=============================================================================#
# make all
#=============================================================================#

all : make_output_dir $(ELF) $(LSS) $(DMP) $(HEX) $(BIN) $(DIS) print_size

# make object files dependent on Makefile
$(OBJS) : Makefile
$(TEST_OBJS) : Makefile
# make .elf file dependent on linker script
$(ELF) : $(LD_SCRIPT)

#-----------------------------------------------------------------------------#
# linking - objects -> elf
#-----------------------------------------------------------------------------#

$(ELF) : $(OBJS)
	@echo 'Linking target: $(ELF)'
	$(CC) $(LD_FLAGS_F) $(OBJS) $(LIBS) -o $@
	@echo ' '

#-----------------------------------------------------------------------------#
# compiling - C source -> objects
#-----------------------------------------------------------------------------#

$(OUT_DIR_F)%.o : %.$(C_EXT)
	@echo 'Compiling file: $<'
	$(CC) -c $(C_FLAGS_F) $< -o $@
	@echo ' '

#-----------------------------------------------------------------------------#
# assembling - ASM source -> objects
#-----------------------------------------------------------------------------#

$(OUT_DIR_F)%.o : %.$(AS_EXT)
	@echo 'Assembling file: $<'
	$(AS) -c $(AS_FLAGS_F) $< -o $@
	@echo ' '

#-----------------------------------------------------------------------------#
# memory images - elf -> hex, elf -> bin
#-----------------------------------------------------------------------------#

$(HEX) : $(ELF)
	@echo 'Creating IHEX image: $(HEX)'
	$(OBJCOPY) -O ihex $< $@
	@echo ' '

$(BIN) : $(ELF)
	@echo 'Creating binary image: $(BIN)'
	$(OBJCOPY) -O binary $< $@
	@echo ' '

#-----------------------------------------------------------------------------#
# memory dump - elf -> dmp
#-----------------------------------------------------------------------------#

$(DMP) : $(ELF)
	@echo 'Creating memory dump: $(DMP)'
	$(OBJDUMP) -x --syms $< > $@
	@echo ' '

#-----------------------------------------------------------------------------#
# extended listing - elf -> lss
#-----------------------------------------------------------------------------#

$(LSS) : $(ELF)
	@echo 'Creating extended listing: $(LSS)'
	$(OBJDUMP) -S $< > $@
	@echo ' '

#-----------------------------------------------------------------------------#
# dissassembly - elf -> dis
#-----------------------------------------------------------------------------#

$(DIS) : $(ELF)
	@echo 'Creating Dissassembly: $(DIS)'
	$(OBJDUMP) -d -S $< > $@
	@echo ' '



#-----------------------------------------------------------------------------#
# print the size of the objects and the .elf file
#-----------------------------------------------------------------------------#

print_size :
	@echo 'Size of modules:'
	$(SIZE) -B -t --common $(OBJS) $(USER_OBJS)
	@echo ' '
	@echo 'Size of target .elf file:'
	$(SIZE) -B $(ELF)
	@echo ' '

#-----------------------------------------------------------------------------#
# create the desired output directory
#-----------------------------------------------------------------------------#

make_output_dir :
	$(shell mkdir $(OUT_DIR_F) 2>/dev/null)

#-----------------------------------------------------------------------------#
# Write to flash of chip by copying to enumerated mass storage device
#-----------------------------------------------------------------------------#

copy: all
	@echo "Writing to" $(MOUNT)
	@cp $(BIN) $(MOUNT)

#-----------------------------------------------------------------------------#
# Write to flash of chip with ocd
#-----------------------------------------------------------------------------#

program: all
	@echo "Programming" $(ELF) "with OCD"
	@$(OCD) -s $(OCD_DIR) $(OCD_FLAGS) -c "program $(ELF) verify reset exit"


#-----------------------------------------------------------------------------#
# Open up serial port
#-----------------------------------------------------------------------------#

com:
	@echo "Starting terminal with baud" $(UART_BAUD)
	@picocom -b $(UART_BAUD) $(UART_COM)

#-----------------------------------------------------------------------------#
# Debug
#-----------------------------------------------------------------------------#

debug_start:
	$(OCD) -s $(OCD_DIR) $(OCD_FLAGS) -c "init" -c "reset halt" -c 'sleep 10'

debug:
	$(GDB) -x ./$(PROJECT).gdbinit



#=============================================================================#
# make clean
#=============================================================================#

clean:
ifeq ($(strip $(OUT_DIR_F)), )
	@echo 'Removing all generated output files'
else
	@echo 'Removing all generated output files from output directory: $(OUT_DIR_F)'
endif
ifneq ($(strip $(GENERATED)), )
	$(RM) $(GENERATED)
else
	@echo 'Nothing to remove...'
endif

#=============================================================================#
# global exports
#=============================================================================#

.PHONY: all clean dependents

.SECONDARY:

# include dependancy files
-include $(DEPS)
