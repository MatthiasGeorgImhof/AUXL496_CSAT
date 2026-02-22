# Compiler and flags
CXX = g++
CXXFLAGS = -g -MMD -std=c++20
EXTRA_FLAGS = -Wall -Wextra -pedantic -Werror -Wconversion -fconcepts-diagnostics-depth=2 -Werror=maybe-uninitialized
EXTRA_EXTRA_FLAGS = -Wzero-as-null-pointer-constant -Wsign-conversion
LOGGER_DEFINES = -DLOGGER_ENABLED

STRICT_FLAGS  = $(CXXFLAGS) $(EXTRA_FLAGS) $(EXTRA_EXTRA_FLAGS) $(LOGGER_DEFINES)
RELAXED_FLAGS = $(CXXFLAGS) $(EXTRA_FLAGS) $(LOGGER_DEFINES)
LOOSE_FLAGS = $(CXXFLAGS) -Wall -Wextra -pedantic $(LOGGER_DEFINES)

INCLUDE_PATHS = -I ../Inc \
				-I $(HOME)/Data/github/can/libcanard/libcanard \
				-I $(HOME)/Data/github/can/libudpard/libudpard \
				-I $(HOME)/Data/github/can/libserard_ck/libserard \
				-I $(HOME)/Data/github/can/o1heap/o1heap \
				-I /usr/include/eigen3 \
				-I /usr/include/doctest \
				-I ../Src \
				-I ../3rdParty/SGP4 \
				-I ../OpenCyphal/include \
				-I ../OpenCyphal/target \
				-I 3rdParty

LIBS_PATH = -L $(HOME)/Data/github/can/lib_x86_64
LIBS = -lcanard_fwd -lserard_fwd -ludpard_fwd -lo1heapd

BIN_DIR = bin

# Create all build directories unconditionally
$(shell mkdir -p \
	$(BIN_DIR)/src \
	$(BIN_DIR)/sgp4 \
	$(BIN_DIR)/3rdParty \
	$(BIN_DIR)/mock_hal \
)

# Wildcard discovery
SRC_FILES := $(wildcard ../Src/*.cpp ../3rdParty/SGP4/*.cpp 3rdParty/*.c)
MOCK_HAL_SRC_FILES := $(wildcard ../Src/mock_hal/*.c)
TEST_FILES := $(wildcard Test*.cpp)

# Object file mapping
SRC_OBJECTS := $(patsubst ../Src/%.cpp,$(BIN_DIR)/src/%.o,$(filter ../Src/%,$(SRC_FILES)))
SGP4_OBJECTS := $(patsubst ../3rdParty/SGP4/%.cpp,$(BIN_DIR)/sgp4/%.o,$(filter ../3rdParty/SGP4/%,$(SRC_FILES)))

THIRDPARTY_OBJECTS := $(patsubst 3rdParty/%.c,$(BIN_DIR)/3rdParty/%.o,$(filter 3rdParty/%,$(SRC_FILES)))

MOCK_HAL_OBJECTS := $(patsubst ../Src/mock_hal/%.c,$(BIN_DIR)/mock_hal/%.o,$(MOCK_HAL_SRC_FILES))

TEST_OBJECTS := $(patsubst %.cpp,$(BIN_DIR)/%.o,$(TEST_FILES))
EXECUTABLES := $(patsubst %.cpp,$(BIN_DIR)/%,$(TEST_FILES))

# Relaxed-flag tests
RELAXED_TESTS := 	TestDetumblerSystem \
					TestKalmanFunctionGPS \
					TestKalmanOrientationMagnetic \
					TestKalmanPositionGPS \
					TestLVLHAttitudeTarget \
					TestMagnetorquerActuation \
					TestMagnetorquerDriver \
					TestMagnetorquerSystem \
					TestOrientationService \
					TestQuaternion \
					TestSGP4PositionTracker \
					TestTaskDetumbler \
					TestTaskMagnetorquer \
					TestTaskOrientationService \
					TestTaskPositionService \
				 	TestIMUExtension \
				 	TestMagneticBDotController \
				 	TestNamedVector3f \
				 	TestOrientationTracker \
				 	TestPositionTracker9D \
				 	TestTaskSGP4

# Some SRC files require relaxed flags (Eigen, etc.)
RELAXED_SRC := 		coordinate_rotators.cpp \
					coordinate_transformations.cpp \
					LVLHAttitudeTarget.cpp \
					MLX90640_API.c \
					Quaternion.cpp

# Some SRC third-party files require even looser flags (MLX90640 API with its own warnings)
LOOSE_SRC := 		MLX90640_API.c

# Per-test extra dependencies
EXTRA_OBJS_TestCoordinateTransformations := src/coordinate_transformations.o src/coordinate_rotators.o src/TimeUtils.o
EXTRA_OBJS_TestGNSS := src/GNSS.o src/GNSSCore.o src/coordinate_transformations.o src/TimeUtils.o
EXTRA_OBJS_TestHSClockSwitch := src/HSClockSwitch.o
EXTRA_OBJS_TestIMUExtension := src/coordinate_transformations.o src/coordinate_rotators.o src/TimeUtils.o
EXTRA_OBJS_TestLVLHAttitudeTarget= src/LVLHAttitudeTarget.o src/Quaternion.o src/coordinate_transformations.o src/coordinate_rotators.o src/TimeUtils.o
EXTRA_OBJS_TestMagnetorquerDriver= src/LVLHAttitudeTarget.o src/Quaternion.o src/coordinate_transformations.o src/coordinate_rotators.o src/TimeUtils.o
EXTRA_OBJS_TestMagnetorquerSystem= src/LVLHAttitudeTarget.o src/Quaternion.o src/coordinate_transformations.o src/coordinate_rotators.o src/TimeUtils.o
EXTRA_OBJS_TestMainLoop := src/RegistrationManager.o src/ServiceManager.o src/TaskCheckMemory.o src/TaskBlinkLED.o
EXTRA_OBJS_TestMLX90640AgainstMelexis := 3rdParty/MLX90640_API.o
EXTRA_OBJS_TestMLX90640ImageProcessor := src/MLX90640ImageProcessor.o
EXTRA_OBJS_TestOrientationService := src/TimeUtils.o src/coordinate_transformations.o src/coordinate_rotators.o src/Quaternion.o
EXTRA_OBJS_TestOrientationTracker := src/TimeUtils.o src/coordinate_transformations.o src/coordinate_rotators.o src/Quaternion.o
EXTRA_OBJS_TestPositionTracker9D := src/TimeUtils.o src/coordinate_transformations.o src/coordinate_rotators.o src/GNSSCore.o src/GNSS.o
EXTRA_OBJS_TestProcessRxQueue := src/ServiceManager.o src/RegistrationManager.o
EXTRA_OBJS_TestQuaternion := src/Quaternion.o
EXTRA_OBJS_TestRegistrationManager := src/ServiceManager.o src/RegistrationManager.o
EXTRA_OBJS_TestServiceManager := src/ServiceManager.o
EXTRA_OBJS_TestSGP4 := sgp4/SGP4.o src/sgp4_tle.o src/TimeUtils.o src/coordinate_transformations.o src/coordinate_rotators.o src/RegistrationManager.o
EXTRA_OBJS_TestSGP4TLE := src/sgp4_tle.o 
EXTRA_OBJS_TestSubscriptionManager := src/RegistrationManager.o
EXTRA_OBJS_TestTaskBlinkLED := src/TaskBlinkLED.o src/RegistrationManager.o
EXTRA_OBJS_TestTaskCheckMemory := src/TaskCheckMemory.o src/RegistrationManager.o
EXTRA_OBJS_TestTaskDetumbler := src/RegistrationManager.o
EXTRA_OBJS_TestTaskMagnetorquer := src/TimeUtils.o src/coordinate_transformations.o src/coordinate_rotators.o src/Quaternion.o src/RegistrationManager.o src/LVLHAttitudeTarget.o
EXTRA_OBJS_TestTaskMLX90640 := src/RegistrationManager.o
EXTRA_OBJS_TestTaskOrientationService := src/RegistrationManager.o src/Quaternion.o src/TimeUtils.o 
EXTRA_OBJS_TestTaskPositionService := src/RegistrationManager.o src/TimeUtils.o src/coordinate_transformations.o src/coordinate_rotators.o src/GNSS.o src/GNSSCore.o sgp4/SGP4.o src/sgp4_tle.o 
EXTRA_OBJS_TestTaskProcessHeartBeat := src/RegistrationManager.o
EXTRA_OBJS_TestTaskProcessTimeSynchronization := src/RegistrationManager.o src/TimeUtils.o
EXTRA_OBJS_TestTaskRegisterServer := src/RegistrationManager.o src/cyphal.o
EXTRA_OBJS_TestTaskRegistrationSubscription := src/RegistrationManager.o src/ServiceManager.o
EXTRA_OBJS_TestTaskRequestGetInfo := src/ServiceManager.o src/RegistrationManager.o
EXTRA_OBJS_TestTaskRequestRead := src/RegistrationManager.o
EXTRA_OBJS_TestTaskRequestWrite := src/RegistrationManager.o
EXTRA_OBJS_TestTaskRespondGetInfo := src/ServiceManager.o src/RegistrationManager.o
EXTRA_OBJS_TestTaskRespondWrite := src/RegistrationManager.o
EXTRA_OBJS_TestTaskSendHeartBeat := src/RegistrationManager.o
EXTRA_OBJS_TestTaskSendNodePortList := src/TaskCheckMemory.o src/TaskBlinkLED.o src/RegistrationManager.o
EXTRA_OBJS_TestTaskSendTimeSynchronization := src/RegistrationManager.o src/TimeUtils.o
EXTRA_OBJS_TestTaskSetRTC := src/RegistrationManager.o src/TimeUtils.o
EXTRA_OBJS_TestTaskSGP4 := sgp4/SGP4.o src/sgp4_tle.o src/RegistrationManager.o src/TimeUtils.o src/coordinate_transformations.o src/coordinate_rotators.o 
EXTRA_OBJS_TestTaskSubscribeNodePortList := src/ServiceManager.o src/RegistrationManager.o
EXTRA_OBJS_TestTimeUtils := src/TimeUtils.o 

ALL_SRC_OBJECTS := $(SRC_OBJECTS) $(SGP4_OBJECTS) $(THIRDPARTY_OBJECTS) $(MOCK_HAL_OBJECTS)

# Default target
all: nunavut $(ALL_SRC_OBJECTS) $(BIN_DIR)/libmockhal.a $(EXECUTABLES)

# Nunavut header generation
nunavut:
	@echo "Running Nunavut header generation..."
	$(MAKE) -C ../OpenCyphal -f makefile

# Compile rules with directory prerequisites
$(BIN_DIR)/src/%.o: | $(BIN_DIR)/src
$(BIN_DIR)/sgp4/%.o: | $(BIN_DIR)/sgp4
$(BIN_DIR)/3rdParty/%.o: | $(BIN_DIR)/3rdParty
$(BIN_DIR)/mock_hal/%.o: | $(BIN_DIR)/mock_hal

# Mock HAL static library
$(BIN_DIR)/libmockhal.a: $(MOCK_HAL_OBJECTS)
	$(AR) rcs $@ $^

define choose_flags
$(if $(filter $(notdir $1),$(LOOSE_SRC)),$(LOOSE_FLAGS), \
  $(if $(filter $(notdir $1),$(RELAXED_SRC) $(addsuffix .cpp,$(RELAXED_TESTS))),$(RELAXED_FLAGS), \
	$(STRICT_FLAGS)))
endef

# Compile rules
$(BIN_DIR)/src/%.o: ../Src/%.cpp
	FLAGS="$(call choose_flags,$<)"; \
	$(CXX) -c $< $$FLAGS $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/sgp4/%.o: ../3rdParty/SGP4/%.cpp
	FLAGS="$(call choose_flags,$<)"; \
	$(CXX) -c $< $$FLAGS $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/3rdParty/%.o: 3rdParty/%.c
	FLAGS="$(call choose_flags,$<)"; \
	$(CXX) -c $< $$FLAGS $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/mock_hal/%.o: ../Src/mock_hal/%.c
	$(CXX) -c $< $(STRICT_FLAGS) $(INCLUDE_PATHS) -o $@

$(BIN_DIR)/%.o: %.cpp
	FLAGS="$(call choose_flags,$<)"; \
	$(CXX) -c $< $$FLAGS $(INCLUDE_PATHS) -o $@

# Generic link rule
$(BIN_DIR)/%: $(BIN_DIR)/%.o $(BIN_DIR)/src/Logger.o $(BIN_DIR)/libmockhal.a
	$(CXX) $(LIBS_PATH) -o $@ \
		$(BIN_DIR)/$*.o \
		$(BIN_DIR)/src/Logger.o \
		$(addprefix $(BIN_DIR)/,$(EXTRA_OBJS_$*)) \
		$(LIBS) \
		$(BIN_DIR)/libmockhal.a

# Pattern: turn every .o into a .d
DEP_FILES := $(ALL_SRC_OBJECTS:.o=.d) \
			 $(TEST_OBJECTS:.o=.d)

# Include dependency files if they exist
-include $(DEP_FILES)

# Cleanup
clean:
	rm -rf $(BIN_DIR)

print-%:
	@echo '$*=$($*)'

.PHONY: all clean nunavut
