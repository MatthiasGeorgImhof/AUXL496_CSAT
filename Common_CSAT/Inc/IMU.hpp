#ifndef __IMU__HPP_
#define __IMU__HPP_

#include <concepts>
#include <optional>
#include <cmath>
#include <array>
#include "au.hpp"

typedef uint8_t ChipID;
typedef std::array<au::QuantityF<au::MetersPerSecondSquaredInBodyFrame>, 3> AccelerationInBodyFrame;
typedef std::array<au::QuantityF<au::MetersPerSecondSquaredInEcefFrame>, 3> AccelerationInEcefFrame;
typedef std::array<au::QuantityF<au::DegreesPerSecondInBodyFrame>, 3> AngularVelocityInBodyFrame;
typedef std::array<au::QuantityF<au::DegreesPerSecondInEcefFrame>, 3> AngularVelocityInEcefFrame;
typedef std::array<au::QuantityF<au::TeslaInBodyFrame>, 3> MagneticFieldInBodyFrame;
typedef std::array<au::QuantityF<au::TeslaInEcefFrame>, 3> MagneticFieldInEcefFrame;
typedef au::QuantityF<au::Celsius> Temperature;

// Concept for readChipID method
template<typename T>
concept ProvidesChipID = requires(T t) {
    { t.readChipID() } -> std::same_as<std::optional<ChipID>>;
};

// Concept for readAccelerometer method
template<typename T>
concept HasBodyAccelerometer = requires(T t) {
    { t.readAccelerometer() } -> std::same_as<std::optional<AccelerationInBodyFrame>>;
};
template<typename T>
concept HasEcefAccelerometer = requires(T t) {
    { t.readAccelerometer() } -> std::same_as<std::optional<AccelerationInEcefFrame>>;
};

// Concept for readGyroscope method
template<typename T>
concept HasBodyGyroscope = requires(T t) {
    { t.readGyroscope() } -> std::same_as<std::optional<AngularVelocityInBodyFrame>>;
};
template<typename T>
concept HasEcefGyroscope = requires(T t) {
    { t.readGyroscope() } -> std::same_as<std::optional<AngularVelocityInEcefFrame>>;
};

// Concept for readMagnetometer method
template<typename T>
concept HasBodyMagnetometer = requires(T t) {
    { t.readMagnetometer() } -> std::same_as<std::optional<MagneticFieldInBodyFrame>>;
};
template<typename T>
concept HasEcefMagnetometer = requires(T t) {
    { t.readMagnetometer() } -> std::same_as<std::optional<MagneticFieldInEcefFrame>>;
};

// Concept for readTemperature method
template<typename T>
concept HasThermometer = requires(T t) {
    { t.readThermometer() } -> std::same_as<std::optional<Temperature>>;
};

#endif // __IMU__HPP_