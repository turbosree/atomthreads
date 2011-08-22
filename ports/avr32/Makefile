############
# Settings #
############

# Build all test applications:
#   make
#
# Program a test application using UISP (appname => test app e.g. sems1):
#   make program app=appname

# Add your tool chain path here
TOOL_CHAIN=/home/sreejithn/Embedded/ATngw100/as4e-ide/plugins/com.atmel.avr.toolchains.linux.x86_3.0.0.201009140852/os/linux/x86

# Location of build tools and atomthreads sources
KERNEL_DIR=../../kernel
TESTS_DIR=../../tests
AVR32_INC_DIR=$(TOOL_CHAIN)/avr32/include
INTC_DRIVER_DIR=./driver/INTC
PM_DRIVER_DIR=./driver/PM
TC_DRIVER_DIR=./driver/TC

CC=$(TOOL_CHAIN)/bin/avr32-gcc 
OBJCOPY=$(TOOL_CHAIN)/bin/avr32-objcopy 
SIZE=$(TOOL_CHAIN)/bin/avr32-size
# UISP=$(TOOL_CHAIN)/

# Modify this to the device name of the UART used for UISP
# UISP_DEV=/dev/ttyUSB0

# Modify this to the CPU you are using
PART=ap7000

# Enable stack-checking. WARNING: the full automated test suite currently
# requires a little over 1KB RAM with stack-checking enabled. If you are 
# using a device with 1KB internal SRAM and no external SRAM then you
# must disable stack-checking to run all of the automated tests.
#STACK_CHECK=true

# Directory for built objects
BUILD_DIR=build

# Port/application object files
APP_OBJECTS = atomport.o tests-main.o #SN: TBD - implement uart driver: uart.o 
APP_ASM_OBJECTS = atomport-asm.o
INTC_DRIVER_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(INTC_DRIVER_DIR)/*.c)))
INTC_DRIVER_ASM_OBJECTS = exception.o
PM_DRIVER_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(PM_DRIVER_DIR)/*.c)))
TC_DRIVER_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(TC_DRIVER_DIR)/*.c)))

# Kernel object files
KERNEL_OBJECTS = atomkernel.o atomsem.o atommutex.o atomtimer.o atomqueue.o

# Collection of built objects (excluding test applications)
ALL_OBJECTS = $(APP_OBJECTS) $(APP_ASM_OBJECTS) $(KERNEL_OBJECTS) $(INTC_DRIVER_OBJECTS) $(INTC_DRIVER_ASM_OBJECTS) $(PM_DRIVER_OBJECTS) $(TC_DRIVER_OBJECTS) 
BUILT_OBJECTS = $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJECTS))

# SN: TBD - Implement tests
# Test object files (dealt with separately as only one per application build)
TEST_OBJECTS = $(notdir $(patsubst %.c,%.o,$(wildcard $(TESTS_DIR)/*.c)))

# Target application filenames (.elf and .hex) for each test object
TEST_ELFS = $(patsubst %.o,%.elf,$(TEST_OBJECTS))
TEST_HEXS = $(patsubst %.o,%.hex,$(TEST_OBJECTS))

# Search build/output directory for dependencies
vpath %.o ./$(BUILD_DIR)
vpath %.elf ./$(BUILD_DIR)
vpath %.hex ./$(BUILD_DIR)

# GCC flags
CFLAGS=-DBOARD=STK1000 -O0 -g3 -Wall -fmessage-length=0 -mpart=$(PART) -ffunction-sections -masm-addr-pseudos

# Enable stack-checking (disable if not required)
ifeq ($(STACK_CHECK),true)
CFLAGS += -DATOM_STACK_CHECKING
endif


#################
# Build targets #
#################

# All tests
all: $(BUILD_DIR) $(TEST_HEXS) Makefile

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Test HEX files (one application build for each test)
$(TEST_HEXS): %.hex: %.elf
	@echo Building $@
	$(OBJCOPY) -j .text -j .data -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@

# Test ELF files (one application build for each test)
$(TEST_ELFS): %.elf: %.o $(KERNEL_OBJECTS) $(APP_OBJECTS) $(APP_ASM_OBJECTS) $(INTC_DRIVER_OBJECTS) $(INTC_DRIVER_ASM_OBJECTS) $(PM_DRIVER_OBJECTS) $(TC_DRIVER_OBJECTS)
	$(CC) $(CFLAGS) $(BUILD_DIR)/$(notdir $<) $(BUILT_OBJECTS) --output $(BUILD_DIR)/$@ -Wl,-Map,$(BUILD_DIR)/$(basename $@).map

# Kernel objects builder
$(KERNEL_OBJECTS): %.o: $(KERNEL_DIR)/%.c
	$(CC) -c $(CFLAGS) -I. -I$(AVR32_INC_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Test objects builder
$(TEST_OBJECTS): %.o: $(TESTS_DIR)/%.c
	$(CC) -c $(CFLAGS) -I. -I$(KERNEL_DIR) -I$(AVR32_INC_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Application C objects builder
$(APP_OBJECTS): %.o: ./%.c
	$(CC) -c $(CFLAGS) -I. -I$(KERNEL_DIR) -I$(AVR32_INC_DIR) -I$(TESTS_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# Application asm objects builder
$(APP_ASM_OBJECTS): %.o: ./%.s
	$(CC) -c $(CFLAGS) -x assembler-with-cpp -I. -I$(KERNEL_DIR) -I$(AVR32_INC_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# INTC_DRIVER asm objects builder
$(INTC_DRIVER_OBJECTS): %.o: $(INTC_DRIVER_DIR)/%.c 
	$(CC) -c $(CFLAGS) -I. -I$(INTC_DRIVER_DIR) -I$(AVR32_INC_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# INTC_DRIVER_ASM objects builder
$(INTC_DRIVER_ASM_OBJECTS): %.o: $(INTC_DRIVER_DIR)/%.s
	$(CC) -c $(CFLAGS) -x assembler-with-cpp -I. -I$(INTC_DRIVER_DIR) -I$(AVR32_INC_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# PM_DRIVER asm objects builder
$(PM_DRIVER_OBJECTS): %.o: $(PM_DRIVER_DIR)/%.c 
	$(CC) -c $(CFLAGS) -I. -I$(PM_DRIVER_DIR) -I$(AVR32_INC_DIR) $< -o $(BUILD_DIR)/$(notdir $@)

# TC_DRIVER asm objects builder
$(TC_DRIVER_OBJECTS): %.o: $(TC_DRIVER_DIR)/%.c 
	$(CC) -c $(CFLAGS) -I. -I$(TC_DRIVER_DIR) -I$(AVR32_INC_DIR) $< -o $(BUILD_DIR)/$(notdir $@) 

# .lst file builder
%.lst: %.c
	$(CC) $(CFLAGS) -I. -I$(KERNEL_DIR) -I$(AVR32_INC_DIR) -I$(TESTS_DIR) -I$(INTC_DRIVER_DIR) -I$(PM_DRIVER_DIR) -I$(TC_DRIVER_DIR) -Wa,-al $< > $@

# Clean
clean:
	rm -f *.o *.elf *.map *.hex *.bin *.lst
	rm -rf doxygen-kernel
	rm -rf doxygen-avr
	rm -rf build

# Send to STK500
# program : $(BUILD_DIR)/$(app).hex
# 	$(SIZE) -C --mcu=$(PART) $(BUILD_DIR)/$(app).elf
# 	$(UISP) -dprog=stk500 -dserial=$(UISP_DEV) -dpart=$(PART) --erase --upload --verify if=$(BUILD_DIR)/$(app).hex

doxygen:
	doxygen $(KERNEL_DIR)/Doxyfile
	doxygen ./Doxyfile
