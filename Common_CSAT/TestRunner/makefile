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
				-I /usr/include/doctest

LIBS_PATH =			-L $(HOME)/Data/github/can/lib_x86_64
LIBS =				-lcanardd -lserardd -ludpardd -lo1heapd

# Source and object files
TEST_FILES = TestArrayList.cpp \
			 TestCyphal.cpp \
			 TestLogger.cpp \
			 TestLoggerCyphal.cpp \
			 TestMockHal.cpp \
			 TestO1HeapAllocator.cpp \
			 TestProcessRxQueue.cpp \
			 TestRegistrationManager.cpp \
			 TestSubscriptionManager.cpp \
			 TestServiceManager.cpp \
			 TestTask.cpp \
			 TestTaskBlinkLED.cpp \
			 TestTaskCheckMemory.cpp \
			 TestTaskSendHeartBeat.cpp \
			 TestTaskSendNodePortList.cpp \
			 TestTaskSubscribeNodePortList.cpp \
             TestBoxSet.cpp \
             TestBoxSetIterator.cpp \
             TestCircularBuffer.cpp \
             TestMainLoop.cpp \
			 TestCyphalSubscriptions.cpp

SRC_FILES = ../Src/Logger.cpp \
            ../Src/RegistrationManager.cpp \
            ../Src/ServiceManager.cpp \
			../Src/TaskBlinkLED.cpp \
			../Src/TaskCheckMemory.cpp \
            ../Src/mock_hal.c

# Extract the names for the targets from the source files
TEST_NAMES = $(TEST_FILES:.cpp=)

# Build directories
BIN_DIR = bin

# Object files
OBJECTS = $(addprefix $(BIN_DIR)/,$(TEST_NAMES:=.o))
SRC_OBJECTS = $(addprefix $(BIN_DIR)/, $(SRC_FILES:.cpp=.o))
SRC_OBJECTS := $(SRC_OBJECTS:.c=.o) # correct for the c source files
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


# Rule for TestMockHal with extra sources
$(BIN_DIR)/TestMockHal.o : TestMockHal.cpp ../Src/mock_hal.c
	@echo "Compiling $< with extra sources"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMockHal : $(BIN_DIR)/TestMockHal.o $(BIN_DIR)/mock_hal.o # Link the object file
	$(CXX) $(BIN_DIR)/TestMockHal.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

$(BIN_DIR)/mock_hal.o : ../Src/mock_hal.c
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestTask with extra sources
$(BIN_DIR)/TestTask.o : TestTask.cpp ../Src/mock_hal.c
	@echo "Compiling $< with extra sources"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTask : $(BIN_DIR)/TestTask.o $(BIN_DIR)/mock_hal.o # Link the object file
	$(CXX) $(BIN_DIR)/TestTask.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Rule for TestLogger with extra sources
$(BIN_DIR)/TestLogger.o : TestLogger.cpp
	@echo "Compiling $< with extra sources"
	$(CXX) $(LOGGER_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestLogger : $(BIN_DIR)/TestLogger.o $(BIN_DIR)/Logger.o $(BIN_DIR)/mock_hal.o
	$(CXX) $(BIN_DIR)/TestLogger.o $(BIN_DIR)/Logger.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

$(BIN_DIR)/Logger.o : ../Src/Logger.cpp
	$(CXX) $(LOGGER_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@


# Rule for TestLoggerCyphal with extra sources
$(BIN_DIR)/TestLoggerCyphal.o : TestLoggerCyphal.cpp
	@echo "Compiling $< with extra sources"
	$(CXX) $(LOGGER_CYPHAL_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestLoggerCyphal : $(BIN_DIR)/TestLoggerCyphal.o $(BIN_DIR)/LoggerCyphal.o $(BIN_DIR)/mock_hal.o
	$(CXX) $(BIN_DIR)/TestLoggerCyphal.o $(BIN_DIR)/LoggerCyphal.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

$(BIN_DIR)/LoggerCyphal.o : ../Src/Logger.cpp
	$(CXX) $(LOGGER_CYPHAL_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@


# Rule for TestServiceManager with extra sources
$(BIN_DIR)/TestServiceManager.o : TestServiceManager.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestServiceManager : $(BIN_DIR)/TestServiceManager.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/mock_hal.o
	$(CXX) $(BIN_DIR)/TestServiceManager.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

$(BIN_DIR)/ServiceManager.o : ../Src/ServiceManager.cpp
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestTaskSendHeartBeat with extra sources
$(BIN_DIR)/TestTaskSendHeartBeat.o : TestTaskSendHeartBeat.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSendHeartBeat : $(BIN_DIR)/TestTaskSendHeartBeat.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o
	$(CXX) $(BIN_DIR)/TestTaskSendHeartBeat.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Rule for TestTaskSendNodePortList with extra sources
$(BIN_DIR)/TestTaskSendNodePortList.o : TestTaskSendNodePortList.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSendNodePortList : $(BIN_DIR)/TestTaskSendNodePortList.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o
	$(CXX) $(BIN_DIR)/TestTaskSendNodePortList.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Rule for TestTaskSubscribeNodePortList with extra sources
$(BIN_DIR)/TestTaskSubscribeNodePortList.o : TestTaskSubscribeNodePortList.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSubscribeNodePortList : $(BIN_DIR)/TestTaskSubscribeNodePortList.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o
	$(CXX) $(BIN_DIR)/TestTaskSubscribeNodePortList.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Rule for TestTaskBlinkLED with extra sources
$(BIN_DIR)/TestTaskBlinkLED.o : TestTaskBlinkLED.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TaskBlinkLED.o : ../Src/TaskBlinkLED.cpp
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskBlinkLED : $(BIN_DIR)/TestTaskBlinkLED.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o
	$(CXX) $(BIN_DIR)/TestTaskBlinkLED.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Rule for TestTaskCheckMemory with extra sources
$(BIN_DIR)/TestTaskCheckMemory.o : TestTaskCheckMemory.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TaskCheckMemory.o : ../Src/TaskCheckMemory.cpp
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskCheckMemory : $(BIN_DIR)/TestTaskCheckMemory.o $(BIN_DIR)/mock_hal.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TaskCheckMemory.o 
	$(CXX) $(BIN_DIR)/TestTaskCheckMemory.o $(BIN_DIR)/mock_hal.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TaskCheckMemory.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Rule for TestMainLoop with extra sources
$(BIN_DIR)/TestMainLoop.o : TestMainLoop.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMainLoop : $(BIN_DIR)/TestMainLoop.o $(BIN_DIR)/mock_hal.o $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o
	$(CXX) $(BIN_DIR)/TestMainLoop.o $(BIN_DIR)/mock_hal.o $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Rule for TestProcessRxQueue with extra sources
$(BIN_DIR)/TestProcessRxQueue.o : TestProcessRxQueue.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestProcessRxQueue : $(BIN_DIR)/TestProcessRxQueue.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o $(BIN_DIR)/ServiceManager.o 
	$(CXX) $(BIN_DIR)/TestProcessRxQueue.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Rule for TestRegistrationManager with extra sources
$(BIN_DIR)/TestRegistrationManager.o : TestRegistrationManager.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/RegistrationManager.o : ../Src/RegistrationManager.cpp
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestRegistrationManager : $(BIN_DIR)/TestRegistrationManager.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o
	$(CXX) $(BIN_DIR)/TestRegistrationManager.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Rule for TestSubscriptionManager with extra sources
$(BIN_DIR)/TestSubscriptionManager.o : TestSubscriptionManager.cpp
	@echo "Compiling $< with extra sources"
	$(CXX)  -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestSubscriptionManager : $(BIN_DIR)/TestSubscriptionManager.o $(BIN_DIR)/mock_hal.o
	$(CXX) $(BIN_DIR)/TestSubscriptionManager.o $(BIN_DIR)/mock_hal.o $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

# Generic rule for all other tests
$(BIN_DIR)/%.o : %.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/% : $(BIN_DIR)/%.o
	$(CXX) $<  $(CXXFLAGS) $(INCLUDE_PATHS) $(LIBS_PATH) $(LIBS) -o $@

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