CXX = g++
EXTRA_CXXFLAGS = -Wall -Wextra -pedantic -Werror
CXXFLAGS = -g -MMD -std=c++20 $(EXTRA_CXXFLAGS)

INCLUDE_PATHS = -I ../Inc \
				-I $(HOME)/Data/github/can/libcanard/libcanard \
				-I $(HOME)/Data/github/can/libudpard/libudpard \
				-I $(HOME)/Data/github/can/libserard_ck/libserard \
				-I $(HOME)/Data/github/can/o1heap/o1heap \
				-I $(HOME)/Data/github/can/nunavut_out \
				-I /usr/include/eigen3 \
				-I /usr/include/doctest \
				-I tests \
				-I ../Src  # Include path for mock_hal headers + all source headers

LIBS_PATH =			-L $(HOME)/Data/github/can/lib_x86_64
LIBS =				-lcanardd -lserardd -ludpardd -lo1heapd

# Source and object files
TEST_FILES = TestArrayList.cpp \
			 TestBoxSet.cpp \
			 TestBoxSetIterator.cpp \
			 TestCachedImageBuffer.cpp \
			 TestCameraSwitch.cpp \
			 TestCircularBuffer.cpp \
			 TestCyphal.cpp \
			 TestCyphalSubscriptions.cpp \
			 TestHalUtil.cpp \
			 TestImageBuffer.cpp \
			 TestImageBufferAccessor.cpp \
			 TestLogger.cpp \
			 TestLoggerCyphal.cpp \
			 TestMainLoop.cpp \
			 TestMockHal.cpp \
			 TestO1HeapAllocator.cpp \
			 TestPowerMonitor.cpp \
			 TestPowerSwitch.cpp \
			 TestProcessRxQueue.cpp \
			 TestRegistrationManager.cpp \
			 TestServiceManager.cpp \
			 TestSubscriptionManager.cpp \
			 TestTask.cpp \
			 TestTaskBlinkLED.cpp \
			 TestTaskCheckMemory.cpp \
			 TestTaskRegistrationSubscription.cpp \
			 TestTaskRequestGetInfo.cpp \
			 TestTaskRespondGetInfo.cpp \
			 TestTaskSendHeartBeat.cpp \
			 TestTaskSendNodePortList.cpp \
			 TestTaskSubscribeNodePortList.cpp

# HAL Test Files
HAL_TEST_FILES =   tests/test_hal_can.cpp \
				   tests/test_hal_gpio.cpp \
				   tests/test_hal_spi.cpp \
				   tests/test_hal_usb.cpp \
				   tests/test_hal_dcmi.cpp \
				   tests/test_hal_i2c.cpp \
				   tests/test_hal_uart.cpp \
				   tests/test_hal_misc.cpp 

SRC_FILES = ../Src/Logger.cpp \
			../Src/RegistrationManager.cpp \
			../Src/ServiceManager.cpp \
			../Src/TaskBlinkLED.cpp \
			../Src/TaskCheckMemory.cpp \
			../Src/ImageBuffer.cpp

# Mock HAL source files
MOCK_HAL_SRC_FILES =   ../Src/mock_hal.c \
					   ../Src/mock_hal/mock_hal_can.c \
					   ../Src/mock_hal/mock_hal_core.c \
					   ../Src/mock_hal/mock_hal_dcmi.c \
					   ../Src/mock_hal/mock_hal_gpio.c \
					   ../Src/mock_hal/mock_hal_i2c.c \
					   ../Src/mock_hal/mock_hal_mem.c \
					   ../Src/mock_hal/mock_hal_spi.c \
					   ../Src/mock_hal/mock_hal_time.c \
					   ../Src/mock_hal/mock_hal_uart.c \
					   ../Src/mock_hal/mock_hal_usb.c

# Extract the names for the targets from the source files
TEST_NAMES = $(TEST_FILES:.cpp=)
HAL_TEST_NAMES = $(HAL_TEST_FILES:.cpp=)

# Build directories
BIN_DIR = bin

# Object files
OBJECTS = $(addprefix $(BIN_DIR)/, $(TEST_NAMES:=.o))
TEST_HAL_OBJECTS = $(addprefix $(BIN_DIR)/, $(HAL_TEST_NAMES:=.o))
SRC_OBJECTS = $(addprefix $(BIN_DIR)/, $(SRC_FILES:.cpp=.o))

# Mock HAL Object Files
MOCK_HAL_SRC_DIRS = $(dir $(MOCK_HAL_SRC_FILES))
MOCK_HAL_OBJECTS = $(addprefix $(BIN_DIR)/, $(MOCK_HAL_SRC_FILES:../Src/%.c=%.o))


# Executables
EXECUTABLES = $(addprefix $(BIN_DIR)/,$(TEST_NAMES))

# Extra defines
LOGGER_EXTRA_DEFINES = -DLOGGER_ENABLED -DLOGGER_OUTPUT_STDERR -DLOGGER_OUTPUT_UART -DLOGGER_OUTPUT_USB
LOGGER_CYPHAL_EXTRA_DEFINES = -DLOGGER_ENABLED -DLOGGER_OUTPUT_CYPHAL

# Default target
all: create_bin_dir $(EXECUTABLES)

# Create the bin directory
create_bin_dir:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(BIN_DIR)/tests
	@mkdir -p $(BIN_DIR)/mock_hal

# Compile C source files mock_hal_*.c
# The corrected rule is below
$(BIN_DIR)/mock_hal.o : ../Src/mock_hal.c
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/mock_hal/%.o : ../Src/mock_hal/%.c
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $(BIN_DIR)/mock_hal/$(notdir $@)

# Compile the HAL test sources
$(BIN_DIR)/%.o: tests/%.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestMockHal with extra sources
$(BIN_DIR)/TestMockHal.o : TestMockHal.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMockHal : $(BIN_DIR)/TestMockHal.o $(TEST_HAL_OBJECTS) $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMockHal.o $(TEST_HAL_OBJECTS) $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestCyphal with extra sources
$(BIN_DIR)/TestCyphal.o : TestCyphal.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestCyphal : $(BIN_DIR)/TestCyphal.o $(MOCK_HAL_OBJECTS) # Link the object file
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestCyphal.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestCameraSwitch with extra sources
$(BIN_DIR)/TestCameraSwitch.o : TestCameraSwitch.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestCameraSwitch : $(BIN_DIR)/TestCameraSwitch.o $(MOCK_HAL_OBJECTS) # Link the object file
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestCameraSwitch.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestImageBufferAccessor.cpp with extra sources
$(BIN_DIR)/TestImageBufferAccessor.o : TestImageBufferAccessor.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestImageBufferAccessor : $(BIN_DIR)/TestImageBufferAccessor.o $(MOCK_HAL_OBJECTS) # Link the object file
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestImageBufferAccessor.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTask with extra sources
$(BIN_DIR)/TestTask.o : TestTask.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTask : $(BIN_DIR)/TestTask.o $(MOCK_HAL_OBJECTS) # Link the object file
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTask.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestLogger with extra sources
$(BIN_DIR)/TestLogger.o : TestLogger.cpp
	@echo "Compiling $<"
	$(CXX) $(LOGGER_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestLogger : $(BIN_DIR)/TestLogger.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestLogger.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(LIBS)

$(BIN_DIR)/Logger.o : ../Src/Logger.cpp
	@echo "Compiling $<"
	$(CXX) $(LOGGER_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestLoggerCyphal with extra sources
$(BIN_DIR)/TestLoggerCyphal.o : TestLoggerCyphal.cpp
	@echo "Compiling $<"
	$(CXX) $(LOGGER_CYPHAL_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestLoggerCyphal : $(BIN_DIR)/TestLoggerCyphal.o $(BIN_DIR)/LoggerCyphal.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestLoggerCyphal.o $(BIN_DIR)/LoggerCyphal.o $(MOCK_HAL_OBJECTS) $(LIBS)

$(BIN_DIR)/LoggerCyphal.o : ../Src/Logger.cpp
	@echo "Compiling $<"
	$(CXX) $(LOGGER_CYPHAL_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestServiceManager with extra sources
$(BIN_DIR)/TestServiceManager.o : TestServiceManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestServiceManager : $(BIN_DIR)/TestServiceManager.o $(BIN_DIR)/ServiceManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestServiceManager.o $(BIN_DIR)/ServiceManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

$(BIN_DIR)/ServiceManager.o : ../Src/ServiceManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestTaskRegistrationSubscription with extra sources
$(BIN_DIR)/TestTaskRegistrationSubscription.o : TestTaskRegistrationSubscription.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRegistrationSubscription : $(BIN_DIR)/TestTaskRegistrationSubscription.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/ServiceManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRegistrationSubscription.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/ServiceManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskSendHeartBeat with extra sources
$(BIN_DIR)/TestTaskSendHeartBeat.o : TestTaskSendHeartBeat.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSendHeartBeat : $(BIN_DIR)/TestTaskSendHeartBeat.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSendHeartBeat.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskSendNodePortList with extra sources
$(BIN_DIR)/TestTaskSendNodePortList.o : TestTaskSendNodePortList.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSendNodePortList : $(BIN_DIR)/TestTaskSendNodePortList.o $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSendNodePortList.o $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskSubscribeNodePortList with extra sources
$(BIN_DIR)/TestTaskSubscribeNodePortList.o : TestTaskSubscribeNodePortList.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSubscribeNodePortList : $(BIN_DIR)/TestTaskSubscribeNodePortList.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSubscribeNodePortList.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskRespondGetInfo with extra sources
$(BIN_DIR)/TestTaskRespondGetInfo.o : TestTaskRespondGetInfo.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRespondGetInfo : $(BIN_DIR)/TestTaskRespondGetInfo.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRespondGetInfo.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskRequestGetInfo with extra sources
$(BIN_DIR)/TestTaskRequestGetInfo.o : TestTaskRequestGetInfo.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRequestGetInfo : $(BIN_DIR)/TestTaskRequestGetInfo.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRequestGetInfo.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskBlinkLED with extra sources
$(BIN_DIR)/TestTaskBlinkLED.o : TestTaskBlinkLED.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TaskBlinkLED.o : ../Src/TaskBlinkLED.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskBlinkLED : $(BIN_DIR)/TestTaskBlinkLED.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskBlinkLED.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskCheckMemory with extra sources
$(BIN_DIR)/TestTaskCheckMemory.o : TestTaskCheckMemory.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TaskCheckMemory.o : ../Src/TaskCheckMemory.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskCheckMemory : $(BIN_DIR)/TestTaskCheckMemory.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TaskCheckMemory.o
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskCheckMemory.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TaskCheckMemory.o $(LIBS)

# Rule for TestMainLoop with extra sources
$(BIN_DIR)/TestMainLoop.o : TestMainLoop.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMainLoop : $(BIN_DIR)/TestMainLoop.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMainLoop.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(LIBS)

# Rule for TestProcessRxQueue with extra sources
$(BIN_DIR)/TestProcessRxQueue.o : TestProcessRxQueue.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestProcessRxQueue : $(BIN_DIR)/TestProcessRxQueue.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/ServiceManager.o
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestProcessRxQueue.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestRegistrationManager with extra sources
$(BIN_DIR)/TestRegistrationManager.o : TestRegistrationManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/RegistrationManager.o : ../Src/RegistrationManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestRegistrationManager : $(BIN_DIR)/TestRegistrationManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestRegistrationManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestSubscriptionManager with extra sources
$(BIN_DIR)/TestSubscriptionManager.o : TestSubscriptionManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestSubscriptionManager : $(BIN_DIR)/TestSubscriptionManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestSubscriptionManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestImageBuffer with extra sources
$(BIN_DIR)/TestImageBuffer.o : TestImageBuffer.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestImageBuffer : $(BIN_DIR)/TestImageBuffer.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestImageBuffer.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestCachedImageBuffer with extra sources
$(BIN_DIR)/TestCachedImageBuffer.o : TestCachedImageBuffer.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestCachedImageBuffer : $(BIN_DIR)/TestCachedImageBuffer.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestCachedImageBuffer.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestPowerSwitch with extra sources
$(BIN_DIR)/TestPowerSwitch.o : TestPowerSwitch.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestPowerSwitch : $(BIN_DIR)/TestPowerSwitch.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestPowerSwitch.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestPowerMonitor with extra sources
$(BIN_DIR)/TestPowerMonitor.o : TestPowerMonitor.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestPowerMonitor : $(BIN_DIR)/TestPowerMonitor.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestPowerMonitor.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestHalUtil with extra sources
$(BIN_DIR)/TestHalUtil.o : TestHalUtil.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestHalUtil : $(BIN_DIR)/TestHalUtil.o $(MOCK_HAL_OBJECTS) # Link the object file
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestHalUtil.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Generic rule for all other tests
$(BIN_DIR)/%.o : %.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/% : $(BIN_DIR)/%.o
	@echo "Linking $@"
	$(CXX) $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) -o $@ $< $(LIBS)

# Run all tests target
run_all: all
	@echo "Running all tests..."
	@for exec in $(EXECUTABLES); do \
		echo "Running $$exec..."; \
		$$exec; \
	done

# Clean build files
clean:
	@echo "Cleaning build files..."
	@rm -rf $(BIN_DIR) *.d ../Src/*.o

-include $(OBJECTS:.o=.d)