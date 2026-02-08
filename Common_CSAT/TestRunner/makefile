CXX = g++
CXXFLAGS = -g -MMD -std=c++20
EXTRA_FLAGS = -Wall -Wextra -pedantic -Werror -Wconversion -fconcepts-diagnostics-depth=2 -Werror=maybe-uninitialized
EXTRA_EXTRA_FLAGS = -Wzero-as-null-pointer-constant -Wsign-conversion

INCLUDE_PATHS = -I ../Inc \
				-I $(HOME)/Data/github/can/libcanard/libcanard \
				-I $(HOME)/Data/github/can/libudpard/libudpard \
				-I $(HOME)/Data/github/can/libserard_ck/libserard \
				-I $(HOME)/Data/github/can/o1heap/o1heap \
				-I /usr/include/eigen3 \
				-I /usr/include/doctest \
				-I tests \
				-I ../Src \
				-I ../3rdParty/SGP4 \
				-I ../OpenCyphal/include \
				-I ../OpenCyphal/target 

LIBS_PATH =			-L $(HOME)/Data/github/can/lib_x86_64
LIBS =				-lcanard_fw -lserard_fw -ludpard_fw -lo1heap

# Source and object files
TEST_FILES = TestAccessorEraseAndFormat.cpp \
			 TestArrayList.cpp \
			 TestBlobStore.cpp \
			 TestBMI270_MMC5983.cpp \
			 TestBMI270.cpp \
			 TestBoxSet.cpp \
			 TestBoxSetIterator.cpp \
			 TestBufferState.cpp \
			 TestCameraControls.cpp \
			 TestCameraPowerConverters.cpp \
			 TestCameraPowerRails.cpp \
			 TestCircularBuffer.cpp \
			 TestCoordinateTransformations.cpp \
			 TestCyphal.cpp \
			 TestCyphalSubscriptions.cpp \
			 TestDetumblerSystem.cpp \
			 TestDirectMemoryAccessor.cpp \
			 TestFileAccess.cpp \
			 TestFileSource.cpp \
			 TestGNSS.cpp \
			 TestGpioPin.cpp \
			 TestHalUtil.cpp \
			 TestHeapAllocation.cpp \
			 TestHSClockSwitch.cpp \
			 TestI2CSwitch.cpp \
			 TestIGRFModel.cpp \
			 TestImageBuffer.cpp \
			 TestImageBufferInternals.cpp \
			 TestImageBufferFalseHeaders.cpp \
			 TestImageBufferReconstruction.cpp \
			 TestImageToWritePipeline.cpp \
			 TestIMUExtension.cpp \
			 TestInputOutputStream.cpp \
			 TestKalmanFunctionGPS.cpp \
			 TestKalmanOrientationMagnetic.cpp \
			 TestKalmanPositionGPS.cpp \
			 TestLogger.cpp \
			 TestLoggerCyphal.cpp \
			 TestLVLHAttitudeTarget.cpp \
			 TestMagneticBDotController.cpp \
			 TestMagnetorquerActuation.cpp \
			 TestMagnetorquerDriver.cpp \
			 TestMagnetorquerSystem.cpp \
			 TestMainLoop.cpp \
			 TestMLX90640.cpp \
			 TestMLX90640AgainstMelexis.cpp \
			 TestMLX90640EEPROM.cpp \
			 TestMLX90640ImageProcessor.cpp \
			 TestMMC5983.cpp \
			 TestMockHal.cpp \
			 TestMR25H10.cpp \
			 TestMT29F4G01Accessor.cpp \
			 TestNamedVector3f.cpp \
			 TestO1HeapAllocator.cpp \
			 TestOrientationService.cpp \
			 TestOrientationTracker.cpp \
			 TestOV2640.cpp \
			 TestOV5640.cpp \
			 TestPositionTracker9D.cpp \
			 TestPowerMonitor.cpp \
			 TestPowerSwitch.cpp \
			 TestProcessRxQueue.cpp \
			 TestQuaternion.cpp \
			 TestRegistrationManager.cpp \
			 TestServiceManager.cpp \
			 TestSGP4.cpp \
			 TestSGP4PositionTracker.cpp \
			 TestSGP4TLE.cpp \
			 TestSingleSlotBuffer.cpp \
			 TestSubscriptionManager.cpp \
			 TestTask.cpp \
			 TestTaskBlinkLED.cpp \
			 TestTaskCheckMemory.cpp \
			 TestTaskDetumbler.cpp \
			 TestTaskMagnetorquer.cpp \
			 TestTaskMLX90640.cpp \
			 TestTaskOrientationService.cpp \
			 TestTaskPositionService.cpp \
			 TestTaskProcessHeartBeat.cpp \
			 TestTaskProcessTimeSynchronization.cpp \
			 TestTaskRegisterServer.cpp \
			 TestTaskRegistrationSubscription.cpp \
			 TestTaskRequestGetInfo.cpp \
			 TestTaskRequestRead.cpp \
			 TestTaskRequestWrite.cpp \
			 TestTaskRespondGetInfo.cpp \
			 TestTaskRespondWrite.cpp \
			 TestTaskSendHeartBeat.cpp \
			 TestTaskSendNodePortList.cpp \
			 TestTaskSendTimeSynchronization.cpp \
			 TestTaskSetRTC.cpp \
			 TestTaskSGP4.cpp \
			 TestTaskSyntheticImageGenerator.cpp \
			 TestTaskSubscribeNodePortList.cpp \
			 TestTimeUtils.cpp \
			 TestTransport.cpp \
			 TestTrigger.cpp \
			 TestTrivialImageBuffer.cpp \
			 TestWMMLegendre.cpp \
			 TestWMMModel.cpp

# HAL Test Files
HAL_TEST_FILES =   tests/test_hal_can.cpp \
				   tests/test_hal_clock.cpp \
				   tests/test_hal_dcmi.cpp \
				   tests/test_hal_gpio.cpp \
				   tests/test_hal_i2c.cpp \
				   tests/test_hal_irq.cpp \
				   tests/test_hal_spi.cpp \
				   tests/test_hal_timer.cpp \
				   tests/test_hal_uart.cpp \
				   tests/test_hal_usb.cpp \
				   tests/test_hal_misc.cpp 

SRC_FILES = ../Src/coordiate_transformations.cpp \
			../Src/coordiate_rotators.cpp \
			../Src/cyphal.cpp \
			../Src/GNSS.cpp \
			../Src/GNSSCore.cpp \
			../Src/HSClockSwitch.cpp \
			../Src/ImageBuffer.cpp \
			../Src/irgf_model.cpp \
			../Src/Logger.cpp \
			../Src/LVLHAttitudeTarget.cpp \
			../Src/RegistrationManager.cpp \
			../Src/Quaternion.cpp \
			../Src/ServiceManager.cpp \
			../Src/sgp4_tle.cpp \
			../Src/TaskBlinkLED.cpp \
			../Src/TaskCheckMemory.cpp \
			../Src/TimeUtils.cpp \
			../3rdParty/SGP4/SGP4.cpp \
			3rdParty/MLX90640_API.c

# Mock HAL source files
MOCK_HAL_SRC_FILES =   ../Src/mock_hal.c \
					   ../Src/mock_hal/mock_hal_can.c \
					   ../Src/mock_hal/mock_hal_clock.c \
					   ../Src/mock_hal/mock_hal_core.c \
					   ../Src/mock_hal/mock_hal_dcmi.c \
					   ../Src/mock_hal/mock_hal_gpio.c \
					   ../Src/mock_hal/mock_hal_i2c.c \
					   ../Src/mock_hal/mock_hal_irq.c \
					   ../Src/mock_hal/mock_hal_mem.c \
					   ../Src/mock_hal/mock_hal_rtc.c \
					   ../Src/mock_hal/mock_hal_spi.c \
					   ../Src/mock_hal/mock_hal_time.c \
					   ../Src/mock_hal/mock_hal_timer.c \
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
all: nunavut create_bin_dir $(EXECUTABLES)

# Run Nunavut header generation before building
nunavut:
	@echo "Running Nunavut header generation..."
	$(MAKE) -C ../OpenCyphal -f makefile

# Create the bin directory
create_bin_dir:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(BIN_DIR)/tests
	@mkdir -p $(BIN_DIR)/mock_hal

# Compile C source files mock_hal_*.c
# The corrected rule is below
$(BIN_DIR)/mock_hal.o : ../Src/mock_hal.c
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/mock_hal/%.o : ../Src/mock_hal/%.c
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $(BIN_DIR)/mock_hal/$(notdir $@)

# Compile the HAL test sources
$(BIN_DIR)/%.o: tests/%.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestMockHal with extra sources
$(BIN_DIR)/TestMockHal.o : TestMockHal.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMockHal : $(BIN_DIR)/TestMockHal.o $(TEST_HAL_OBJECTS) $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMockHal.o $(TEST_HAL_OBJECTS) $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestCyphal with extra sources
$(BIN_DIR)/TestCyphal.o : TestCyphal.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestCyphal : $(BIN_DIR)/TestCyphal.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestCyphal.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestBMI270 with extra sources
$(BIN_DIR)/TestBMI270.o : TestBMI270.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestBMI270 : $(BIN_DIR)/TestBMI270.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestBMI270.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestAccessorEraseAndFormat with extra sources
$(BIN_DIR)/TestAccessorEraseAndFormat.o : TestAccessorEraseAndFormat.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestAccessorEraseAndFormat : $(BIN_DIR)/TestAccessorEraseAndFormat.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestAccessorEraseAndFormat.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestBMI270_MMC5983 with extra sources
$(BIN_DIR)/TestBMI270_MMC5983.o : TestBMI270_MMC5983.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestBMI270_MMC5983 : $(BIN_DIR)/TestBMI270_MMC5983.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestBMI270_MMC5983.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestMMC5983 with extra sources
$(BIN_DIR)/TestMMC5983.o : TestMMC5983.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMMC5983 : $(BIN_DIR)/TestMMC5983.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMMC5983.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestMR25H10 with extra sources
$(BIN_DIR)/TestMR25H10.o : TestMR25H10.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMR25H10 : $(BIN_DIR)/TestMR25H10.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMR25H10.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestMLX90640 with extra sources
$(BIN_DIR)/TestMLX90640.o : TestMLX90640.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMLX90640 : $(BIN_DIR)/TestMLX90640.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMLX90640.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestMLX90640AgainstMelexis with extra sources
$(BIN_DIR)/TestMLX90640AgainstMelexis.o : TestMLX90640AgainstMelexis.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -I3rdParty -o $@

$(BIN_DIR)/MLX90640_API.o : 3rdParty/MLX90640_API.c
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(INCLUDE_PATHS) -I3rdParty -o $@

$(BIN_DIR)/TestMLX90640AgainstMelexis : $(BIN_DIR)/TestMLX90640AgainstMelexis.o $(BIN_DIR)/MLX90640_API.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMLX90640AgainstMelexis.o $(BIN_DIR)/MLX90640_API.o $(LIBS)

# Rule for TestMLX90640ImageProcessor with extra sources
$(BIN_DIR)/TestMLX90640ImageProcessor.o : TestMLX90640ImageProcessor.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/MLX90640ImageProcessor.o : ../Src/MLX90640ImageProcessor.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMLX90640ImageProcessor : $(BIN_DIR)/TestMLX90640ImageProcessor.o $(BIN_DIR)/MLX90640ImageProcessor.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMLX90640ImageProcessor.o $(BIN_DIR)/MLX90640ImageProcessor.o $(LIBS)

# Rule for TestMLX90640 with extra sources
$(BIN_DIR)/TestTaskMLX90640.o : TestTaskMLX90640.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskMLX90640 : $(BIN_DIR)/TestTaskMLX90640.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskMLX90640.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestOV2640 with extra sources
$(BIN_DIR)/TestOV2640.o : TestOV2640.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestOV2640 : $(BIN_DIR)/TestOV2640.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestOV2640.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestOV5640 with extra sources
$(BIN_DIR)/TestOV5640.o : TestOV5640.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestOV5640 : $(BIN_DIR)/TestOV5640.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestOV5640.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestGpioPin with extra sources
$(BIN_DIR)/TestGpioPin.o : TestGpioPin.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestGpioPin : $(BIN_DIR)/TestGpioPin.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestGpioPin.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestCameraControls with extra sources
$(BIN_DIR)/TestCameraControls.o : TestCameraControls.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestCameraControls : $(BIN_DIR)/TestCameraControls.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestCameraControls.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestCameraPowerConverters with extra sources
$(BIN_DIR)/TestCameraPowerConverters.o : TestCameraPowerConverters.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestCameraPowerConverters : $(BIN_DIR)/TestCameraPowerConverters.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestCameraPowerConverters.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestCameraPowerRails with extra sources
$(BIN_DIR)/TestCameraPowerRails.o : TestCameraPowerRails.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestCameraPowerRails : $(BIN_DIR)/TestCameraPowerRails.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestCameraPowerRails.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTransport with extra sources
$(BIN_DIR)/TestTransport.o : TestTransport.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTransport : $(BIN_DIR)/TestTransport.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTransport.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestI2CSwitch with extra sources
$(BIN_DIR)/TestI2CSwitch.o : TestI2CSwitch.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestI2CSwitch : $(BIN_DIR)/TestI2CSwitch.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestI2CSwitch.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestHSClockSwitch with extra sources
$(BIN_DIR)/TestHSClockSwitch.o : TestHSClockSwitch.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/HSClockSwitch.o : ../Src/HSClockSwitch.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestHSClockSwitch : $(BIN_DIR)/TestHSClockSwitch.o $(BIN_DIR)/HSClockSwitch.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestHSClockSwitch.o $(BIN_DIR)/HSClockSwitch.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestDirectMemoryAccessor.cpp with extra sources
$(BIN_DIR)/TestDirectMemoryAccessor.o : TestDirectMemoryAccessor.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestDirectMemoryAccessor : $(BIN_DIR)/TestDirectMemoryAccessor.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestDirectMemoryAccessor.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestImageBufferInternals.cpp with extra sources
$(BIN_DIR)/TestImageBufferInternals.o : TestImageBufferInternals.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestImageBufferInternals : $(BIN_DIR)/TestImageBufferInternals.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestImageBufferInternals.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestImageBufferFalseHeaders.cpp with extra sources
$(BIN_DIR)/TestImageBufferFalseHeaders.o : TestImageBufferFalseHeaders.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestImageBufferFalseHeaders : $(BIN_DIR)/TestImageBufferFalseHeaders.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestImageBufferFalseHeaders.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestImageBufferReconstruction.cpp with extra sources
$(BIN_DIR)/TestImageBufferReconstruction.o : TestImageBufferReconstruction.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestImageBufferReconstruction : $(BIN_DIR)/TestImageBufferReconstruction.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestImageBufferReconstruction.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestImageToWritePipeline.cpp with extra sources
$(BIN_DIR)/TestImageToWritePipeline.o : TestImageToWritePipeline.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestImageToWritePipeline : $(BIN_DIR)/TestImageToWritePipeline.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestImageToWritePipeline.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestMT29F4G01Accessor.cpp with extra sources
$(BIN_DIR)/TestMT29F4G01Accessor.o : TestMT29F4G01Accessor.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMT29F4G01Accessor : $(BIN_DIR)/TestMT29F4G01Accessor.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMT29F4G01Accessor.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTask with extra sources
$(BIN_DIR)/TestTask.o : TestTask.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTask : $(BIN_DIR)/TestTask.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTask.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestNamedVector3f with extra sources
$(BIN_DIR)/TestNamedVector3f.o : TestNamedVector3f.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestNamedVector3f : $(BIN_DIR)/TestNamedVector3f.o 
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestNamedVector3f.o $(LIBS)

# Rule for TestLogger with extra sources
$(BIN_DIR)/TestLogger.o : TestLogger.cpp
	@echo "Compiling $<"
	$(CXX) $(LOGGER_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestLogger : $(BIN_DIR)/TestLogger.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestLogger.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(LIBS)

$(BIN_DIR)/Logger.o : ../Src/Logger.cpp
	@echo "Compiling $<"
	$(CXX) $(LOGGER_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestLoggerCyphal with extra sources
$(BIN_DIR)/TestLoggerCyphal.o : TestLoggerCyphal.cpp
	@echo "Compiling $<"
	$(CXX) $(LOGGER_CYPHAL_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestLoggerCyphal : $(BIN_DIR)/TestLoggerCyphal.o $(BIN_DIR)/LoggerCyphal.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestLoggerCyphal.o $(BIN_DIR)/LoggerCyphal.o $(MOCK_HAL_OBJECTS) $(LIBS)

$(BIN_DIR)/LoggerCyphal.o : ../Src/Logger.cpp
	@echo "Compiling $<"
	$(CXX) $(LOGGER_CYPHAL_EXTRA_DEFINES) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestServiceManager with extra sources
$(BIN_DIR)/TestServiceManager.o : TestServiceManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestServiceManager : $(BIN_DIR)/TestServiceManager.o $(BIN_DIR)/ServiceManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestServiceManager.o $(BIN_DIR)/ServiceManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

$(BIN_DIR)/ServiceManager.o : ../Src/ServiceManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

# Rule for TestTaskRegistrationSubscription with extra sources
$(BIN_DIR)/TestTaskRegistrationSubscription.o : TestTaskRegistrationSubscription.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRegistrationSubscription : $(BIN_DIR)/TestTaskRegistrationSubscription.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/ServiceManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRegistrationSubscription.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/ServiceManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskRegisterServer with extra sources
$(BIN_DIR)/TestTaskRegisterServer.o : TestTaskRegisterServer.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/cyphal.o : ../Src/cyphal.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRegisterServer : $(BIN_DIR)/TestTaskRegisterServer.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/cyphal.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRegisterServer.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/cyphal.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskSendHeartBeat with extra sources
$(BIN_DIR)/TestTaskSendHeartBeat.o : TestTaskSendHeartBeat.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSendHeartBeat : $(BIN_DIR)/TestTaskSendHeartBeat.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSendHeartBeat.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskProcessHeartBeat with extra sources
$(BIN_DIR)/TestTaskProcessHeartBeat.o : TestTaskProcessHeartBeat.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskProcessHeartBeat : $(BIN_DIR)/TestTaskProcessHeartBeat.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskProcessHeartBeat.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskSendNodePortList with extra sources
$(BIN_DIR)/TestTaskSendNodePortList.o : TestTaskSendNodePortList.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSendNodePortList : $(BIN_DIR)/TestTaskSendNodePortList.o $(BIN_DIR)/Logger.o $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSendNodePortList.o $(BIN_DIR)/Logger.o $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskSubscribeNodePortList with extra sources
$(BIN_DIR)/TestTaskSubscribeNodePortList.o : TestTaskSubscribeNodePortList.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSubscribeNodePortList : $(BIN_DIR)/TestTaskSubscribeNodePortList.o $(BIN_DIR)/Logger.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSubscribeNodePortList.o $(BIN_DIR)/Logger.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskRespondGetInfo with extra sources
$(BIN_DIR)/TestTaskRespondGetInfo.o : TestTaskRespondGetInfo.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRespondGetInfo : $(BIN_DIR)/TestTaskRespondGetInfo.o $(BIN_DIR)/Logger.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRespondGetInfo.o $(BIN_DIR)/Logger.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskRequestGetInfo with extra sources
$(BIN_DIR)/TestTaskRequestGetInfo.o : TestTaskRequestGetInfo.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRequestGetInfo : $(BIN_DIR)/TestTaskRequestGetInfo.o $(BIN_DIR)/Logger.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRequestGetInfo.o $(BIN_DIR)/Logger.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskRequestWrite with extra sources
$(BIN_DIR)/TestTaskRequestWrite.o : TestTaskRequestWrite.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRequestWrite : $(BIN_DIR)/TestTaskRequestWrite.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRequestWrite.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskRespondWrite with extra sources
$(BIN_DIR)/TestTaskRespondWrite.o : TestTaskRespondWrite.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRespondWrite : $(BIN_DIR)/TestTaskRespondWrite.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRespondWrite.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskRequestRead with extra sources
$(BIN_DIR)/TestTaskRequestRead.o : TestTaskRequestRead.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskRequestRead : $(BIN_DIR)/TestTaskRequestRead.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskRequestRead.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskSendTimeSynchronization with extra sources
$(BIN_DIR)/TestTaskSendTimeSynchronization.o : TestTaskSendTimeSynchronization.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSendTimeSynchronization : $(BIN_DIR)/TestTaskSendTimeSynchronization.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TimeUtils.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSendTimeSynchronization.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TimeUtils.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskProcessTimeSynchronization with extra sources
$(BIN_DIR)/TestTaskProcessTimeSynchronization.o : TestTaskProcessTimeSynchronization.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskProcessTimeSynchronization : $(BIN_DIR)/TestTaskProcessTimeSynchronization.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TimeUtils.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskProcessTimeSynchronization.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TimeUtils.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskDetumbler with extra sources
$(BIN_DIR)/TestTaskDetumbler.o : TestTaskDetumbler.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskDetumbler : $(BIN_DIR)/TestTaskDetumbler.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskDetumbler.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskMagnetorquer with extra sources
$(BIN_DIR)/TestTaskMagnetorquer.o : TestTaskMagnetorquer.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskMagnetorquer : $(BIN_DIR)/TestTaskMagnetorquer.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/coordinate_rotators.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskMagnetorquer.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/coordinate_rotators.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskBlinkLED with extra sources
$(BIN_DIR)/TestTaskBlinkLED.o : TestTaskBlinkLED.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TaskBlinkLED.o : ../Src/TaskBlinkLED.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskBlinkLED : $(BIN_DIR)/TestTaskBlinkLED.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskBlinkLED.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskCheckMemory with extra sources
$(BIN_DIR)/TestTaskCheckMemory.o : TestTaskCheckMemory.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TaskCheckMemory.o : ../Src/TaskCheckMemory.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskCheckMemory : $(BIN_DIR)/TestTaskCheckMemory.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TaskCheckMemory.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskCheckMemory.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/TaskCheckMemory.o $(LIBS)

# Rule for TestTaskSetRTC with extra sources
$(BIN_DIR)/TestTaskSetRTC.o : TestTaskSetRTC.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSetRTC : $(BIN_DIR)/TestTaskSetRTC.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(BIN_DIR)/TimeUtils.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSetRTC.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(BIN_DIR)/TimeUtils.o $(LIBS) 
	@echo "Linking $@"

# Rule for TestTaskSGP4 with extra sources
$(BIN_DIR)/TestTaskSGP4.o : TestTaskSGP4.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/SGP4.o : ../3rdParty/SGP4/SGP4.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSGP4 : $(BIN_DIR)/TestTaskSGP4.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/SGP4.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/sgp4_tle.o $(BIN_DIR)/coordinate_transformations.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSGP4.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/SGP4.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/sgp4_tle.o $(BIN_DIR)/coordinate_transformations.o $(LIBS) 
	@echo "Linking $@"

# Rule for TestSGP4 with extra sources
$(BIN_DIR)/TestSGP4.o : TestSGP4.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestSGP4 : $(BIN_DIR)/TestSGP4.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/SGP4.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/sgp4_tle.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestSGP4.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/SGP4.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/sgp4_tle.o $(LIBS) 
	@echo "Linking $@"

# Rule for TestTaskSyntheticImageGenerator with extra sources
$(BIN_DIR)/TestTaskSyntheticImageGenerator.o : TestTaskSyntheticImageGenerator.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskSyntheticImageGenerator : $(BIN_DIR)/TestTaskSyntheticImageGenerator.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskSyntheticImageGenerator.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestMainLoop with extra sources
$(BIN_DIR)/TestMainLoop.o : TestMainLoop.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMainLoop : $(BIN_DIR)/TestMainLoop.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMainLoop.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/TaskCheckMemory.o $(BIN_DIR)/TaskBlinkLED.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/RegistrationManager.o $(LIBS)

# Rule for TestProcessRxQueue with extra sources
$(BIN_DIR)/TestProcessRxQueue.o : TestProcessRxQueue.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestProcessRxQueue : $(BIN_DIR)/TestProcessRxQueue.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(BIN_DIR)/ServiceManager.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestProcessRxQueue.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/ServiceManager.o $(BIN_DIR)/Logger.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestRegistrationManager with extra sources
$(BIN_DIR)/TestRegistrationManager.o : TestRegistrationManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/RegistrationManager.o : ../Src/RegistrationManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestRegistrationManager : $(BIN_DIR)/TestRegistrationManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestRegistrationManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestSubscriptionManager with extra sources
$(BIN_DIR)/TestSubscriptionManager.o : TestSubscriptionManager.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestSubscriptionManager : $(BIN_DIR)/TestSubscriptionManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestSubscriptionManager.o $(BIN_DIR)/RegistrationManager.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestImageBuffer with extra sources
$(BIN_DIR)/TestImageBuffer.o : TestImageBuffer.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestImageBuffer : $(BIN_DIR)/TestImageBuffer.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestImageBuffer.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestPowerSwitch with extra sources
$(BIN_DIR)/TestPowerSwitch.o : TestPowerSwitch.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestPowerSwitch : $(BIN_DIR)/TestPowerSwitch.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestPowerSwitch.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestPowerMonitor with extra sources
$(BIN_DIR)/TestPowerMonitor.o : TestPowerMonitor.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestPowerMonitor : $(BIN_DIR)/TestPowerMonitor.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestPowerMonitor.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestHalUtil with extra sources
$(BIN_DIR)/TestHalUtil.o : TestHalUtil.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestHalUtil : $(BIN_DIR)/TestHalUtil.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestHalUtil.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestHeapAllocation with extra sources
$(BIN_DIR)/TestHeapAllocation.o : TestHeapAllocation.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestHeapAllocation : $(BIN_DIR)/TestHeapAllocation.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestHeapAllocation.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestCoordinateTransformations with extra sources
$(BIN_DIR)/TestCoordinateTransformations.o : TestCoordinateTransformations.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/coordinate_transformations.o : ../Src/coordinate_transformations.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestCoordinateTransformations : $(BIN_DIR)/TestCoordinateTransformations.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestCoordinateTransformations.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o $(LIBS)

# Rule for TestGNSS with extra sources
$(BIN_DIR)/TestGNSS.o : TestGNSS.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/GNSS.o : ../Src/GNSS.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/GNSSCore.o : ../Src/GNSSCore.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestGNSS : $(BIN_DIR)/TestGNSS.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestGNSS.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestSGO4TLE with extra sources
$(BIN_DIR)/TestSGP4TLE.o : TestSGP4TLE.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/sgp4_tle.o : ../Src/sgp4_tle.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestSGP4TLE : $(BIN_DIR)/TestSGP4TLE.o $(BIN_DIR)/sgp4_tle.o 
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestSGP4TLE.o $(BIN_DIR)/sgp4_tle.o $(LIBS)

# Rule for TestInputOutputStream with extra sources
$(BIN_DIR)/TestInputOutputStream.o : TestInputOutputStream.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestInputOutputStream : $(BIN_DIR)/TestInputOutputStream.o $ $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestInputOutputStream.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTrigger with extra sources
$(BIN_DIR)/TestTrigger.o : TestTrigger.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTrigger : $(BIN_DIR)/TestTrigger.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTrigger.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTimeUtils with extra sources
$(BIN_DIR)/TestTimeUtils.o : TestTimeUtils.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TimeUtils.o : ../Src/TimeUtils.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTimeUtils : $(BIN_DIR)/TestTimeUtils.o $(BIN_DIR)/TimeUtils.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTimeUtils.o $(BIN_DIR)/TimeUtils.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestKalmanPositionGPS with EXTRA_FLAGS only
$(BIN_DIR)/TestKalmanPositionGPS.o : TestKalmanPositionGPS.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestKalmanPositionGPS : $(BIN_DIR)/TestKalmanPositionGPS.o 
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestKalmanPositionGPS.o $(LIBS)

# Rule for TestKalmanFunctionGPS with EXTRA_FLAGS only
$(BIN_DIR)/TestKalmanFunctionGPS.o : TestKalmanFunctionGPS.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestKalmanFunctionGPS : $(BIN_DIR)/TestKalmanFunctionGPS.o 
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestKalmanFunctionGPS.o $(LIBS)

# Rule for TestKalmanOrientationMagnetic with EXTRA_FLAGS only
$(BIN_DIR)/TestKalmanOrientationMagnetic.o : TestKalmanOrientationMagnetic.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestKalmanOrientationMagnetic : $(BIN_DIR)/TestKalmanOrientationMagnetic.o 
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestKalmanOrientationMagnetic.o $(LIBS)

# Rule for TestOrientationTracker with with EXTRA_FLAGS only
$(BIN_DIR)/TestOrientationTracker.o : TestOrientationTracker.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestOrientationTracker : $(BIN_DIR)/TestOrientationTracker.o $(BIN_DIR)/Quaternion.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestOrientationTracker.o $(BIN_DIR)/Quaternion.o $(LIBS)

# Rule for TestOrientationService with with EXTRA_FLAGS only
$(BIN_DIR)/TestOrientationService.o : TestOrientationService.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestOrientationService : $(BIN_DIR)/TestOrientationService.o $(BIN_DIR)/Quaternion.o $(BIN_DIR)/TimeUtils.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestOrientationService.o $(BIN_DIR)/Quaternion.o $(BIN_DIR)/TimeUtils.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestQuaternion with with EXTRA_FLAGS only
$(BIN_DIR)/TestQuaternion.o : TestQuaternion.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/Quaternion.o : ../Src/Quaternion.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestQuaternion : $(BIN_DIR)/TestQuaternion.o $(BIN_DIR)/Quaternion.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestQuaternion.o $(BIN_DIR)/Quaternion.o $(LIBS)

# Rule for TestSGP4PositionTracker with with EXTRA_FLAGS only
$(BIN_DIR)/TestSGP4PositionTracker.o : TestSGP4PositionTracker.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestSGP4PositionTracker : $(BIN_DIR)/TestSGP4PositionTracker.o 
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestSGP4PositionTracker.o $(LIBS)

# Rule for TestPositionTracker9D with with EXTRA_FLAGS only
$(BIN_DIR)/TestPositionTracker9D.o : TestPositionTracker9D.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestPositionTracker9D : $(BIN_DIR)/TestPositionTracker9D.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestPositionTracker9D.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskPositionService with with EXTRA_FLAGS only
$(BIN_DIR)/TestTaskPositionService.o : TestTaskPositionService.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskPositionService : $(BIN_DIR)/TestTaskPositionService.o $(BIN_DIR)/Logger.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/SGP4.o $(BIN_DIR)/sgp4_tle.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/coordinate_rotators.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskPositionService.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/SGP4.o $(BIN_DIR)/sgp4_tle.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/coordinate_rotators.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestTaskOrientationService with with EXTRA_FLAGS only
$(BIN_DIR)/TestTaskOrientationService.o : TestTaskOrientationService.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestTaskOrientationService : $(BIN_DIR)/TestTaskOrientationService.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/Quaternion.o $(BIN_DIR)/SGP4.o $(BIN_DIR)/sgp4_tle.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestTaskOrientationService.o $(BIN_DIR)/Logger.o $(BIN_DIR)/RegistrationManager.o $(BIN_DIR)/GNSS.o $(BIN_DIR)/GNSSCore.o $(BIN_DIR)/TimeUtils.o $(BIN_DIR)/Quaternion.o $(BIN_DIR)/SGP4.o $(BIN_DIR)/sgp4_tle.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestIMUExtension with with EXTRA_FLAGS only
$(BIN_DIR)/TestIMUExtension.o : TestIMUExtension.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestIMUExtension : $(BIN_DIR)/TestIMUExtension.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/TimeUtils.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestIMUExtension.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/TimeUtils.o $(LIBS)

# Rule for TestMagneticBDotController with with EXTRA_FLAGS only
$(BIN_DIR)/TestMagneticBDotController.o : TestMagneticBDotController.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMagneticBDotController : $(BIN_DIR)/TestMagneticBDotController.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMagneticBDotController.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestMagnetorquerDriver with with EXTRA_FLAGS only
$(BIN_DIR)/TestMagnetorquerDriver.o : TestMagnetorquerDriver.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMagnetorquerDriver : $(BIN_DIR)/TestMagnetorquerDriver.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMagnetorquerDriver.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o $(LIBS)

# Rule for TestMagnetorquerActuation with with EXTRA_FLAGS only
$(BIN_DIR)/TestMagnetorquerActuation.o : TestMagnetorquerActuation.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMagnetorquerActuation : $(BIN_DIR)/TestMagnetorquerActuation.o $(MOCK_HAL_OBJECTS)
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMagnetorquerActuation.o $(MOCK_HAL_OBJECTS) $(LIBS)

# Rule for TestLVLHAttitudeTarget with with EXTRA_FLAGS only
$(BIN_DIR)/TestLVLHAttitudeTarget.o : TestLVLHAttitudeTarget.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/LVLHAttitudeTarget.o : ../Src/LVLHAttitudeTarget.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/coordinate_rotators.o : ../Src/coordinate_rotators.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestLVLHAttitudeTarget : $(BIN_DIR)/TestLVLHAttitudeTarget.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestLVLHAttitudeTarget.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o $(LIBS)

# Rule for TestMagnetorquerSystem with with EXTRA_FLAGS only
$(BIN_DIR)/TestMagnetorquerSystem.o : TestMagnetorquerSystem.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestMagnetorquerSystem : $(BIN_DIR)/TestMagnetorquerSystem.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestMagnetorquerSystem.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o $(LIBS)

# Rule for TestDetumblerSystem with with EXTRA_FLAGS only
$(BIN_DIR)/TestDetumblerSystem.o : TestDetumblerSystem.cpp
	@echo "Compiling $<"
	$(CXX)	-c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/TestDetumblerSystem : $(BIN_DIR)/TestDetumblerSystem.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $(BIN_DIR)/TestDetumblerSystem.o $(BIN_DIR)/LVLHAttitudeTarget.o $(BIN_DIR)/coordinate_rotators.o $(BIN_DIR)/coordinate_transformations.o $(BIN_DIR)/TimeUtils.o $(LIBS)

# Generic rule for all other tests
$(BIN_DIR)/%.o : %.cpp
	@echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/% : $(BIN_DIR)/%.o
	@echo "Linking $@"
	$(CXX) $(LIBS_PATH) -o $@ $< $(LIBS)

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