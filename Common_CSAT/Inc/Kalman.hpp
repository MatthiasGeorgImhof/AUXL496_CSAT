#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <array>

// Generic Kalman Filter (Template)
template <int StateSize, int MeasurementSize>
class KalmanFilter
{
public:
    // Matrices for the Kalman Filter calculations
    Eigen::Matrix<float, StateSize, StateSize> processNoiseCovarianceMatrix;
    Eigen::Matrix<float, MeasurementSize, MeasurementSize> measurementNoiseCovarianceMatrix;
    Eigen::Matrix<float, StateSize, StateSize> stateCovarianceMatrix;
    Eigen::Matrix<float, StateSize, 1> stateVector;

    // Constructor for the KalmanFilter
    /**
     * @brief Constructor for the Kalman Filter.
     *
     * @param initialProcessNoiseMatrix The matrix to define the process noise.
     * @param initialMeasurementNoiseMatrix The matrix to define the measurement noise.
     * @param initialStateCovarianceMatrix The matrix to represent initial state uncertainty.
     * @param initialStateVector The vector to represent initial state estimate.
     */
    KalmanFilter(const Eigen::Matrix<float, StateSize, StateSize> &initialProcessNoiseMatrix,
                 const Eigen::Matrix<float, MeasurementSize, MeasurementSize> &initialMeasurementNoiseMatrix,
                 const Eigen::Matrix<float, StateSize, StateSize> &initialStateCovarianceMatrix,
                 const Eigen::Matrix<float, StateSize, 1> &initialStateVector)
        : processNoiseCovarianceMatrix(initialProcessNoiseMatrix),
          measurementNoiseCovarianceMatrix(initialMeasurementNoiseMatrix),
          stateCovarianceMatrix(initialStateCovarianceMatrix),
          stateVector(initialStateVector) {}

    // Method to perform the prediction step of the Kalman Filter
    /**
     * @brief Performs the Kalman Filter prediction step to project the current state to the next step.
     *
     * @param stateTransitionMatrix The matrix to project the current state to the next step.
     *
     * @note This method updates the internal state vector and state covariance matrix.
     */
    void predict(const Eigen::Matrix<float, StateSize, StateSize> &stateTransitionMatrix)
    {
        // std::cout << "Kalman::predict - Before Prediction" << std::endl;
        // std::cout << "stateVector:\n" << stateVector << std::endl;
        // std::cout << "stateTransitionMatrix:\n" << stateTransitionMatrix << std::endl;

        // Project the current state forward using state transition matrix
        Eigen::Matrix<float, StateSize, 1> predictedStateVector = stateTransitionMatrix * stateVector;

        // std::cout << "Kalman::predict - After Prediction" << std::endl;
        // std::cout << "predictedStateVector:\n" << predictedStateVector << std::endl;

        // Project the state covariance forward, adding process noise
        stateCovarianceMatrix = stateTransitionMatrix * stateCovarianceMatrix * stateTransitionMatrix.transpose() + processNoiseCovarianceMatrix;

        // std::cout << "Kalman::predict - After Covariance Update" << std::endl;
        // std::cout << "stateCovarianceMatrix:\n" << stateCovarianceMatrix << std::endl;

        stateVector = predictedStateVector;
    }

    template <typename ControlMatrix, typename ControlVector>
    void predictWithControl(const Eigen::Matrix<float, StateSize, StateSize> &A,
                            const ControlMatrix &B,
                            const ControlVector &u)
    {
        stateVector = A * stateVector + B * u;
        stateCovarianceMatrix = A * stateCovarianceMatrix * A.transpose() + processNoiseCovarianceMatrix;
        static_assert(static_cast<int>(ControlMatrix::ColsAtCompileTime) == static_cast<int>(ControlVector::RowsAtCompileTime),
              "Control matrix columns must match control vector rows");
    }

    // Method to perform the update step of the Kalman Filter
    /**
     * @brief Performs the Kalman Filter update step to refine the state estimate using a measurement.
     *
     * @param measurementMatrix The matrix to indicate which state variables are measured.
     * @param measurementVector The measurement of the current state.
     *
     * @note This method updates the internal state vector and state covariance matrix.
     */
    void update(const Eigen::Matrix<float, MeasurementSize, StateSize> &measurementMatrix,
                const Eigen::Matrix<float, MeasurementSize, 1> &measurementVector)
    {

        // std::cout << "Kalman::update - Before Update" << std::endl;
        // std::cout << "stateVector:\n"<< stateVector << std::endl;
        // std::cout << "stateCovarianceMatrix:\n" << stateCovarianceMatrix << std::endl;
        // std::cout << "measurementMatrix:\n" << measurementMatrix << std::endl;
        // std::cout << "measurementVector:\n" << measurementVector << std::endl;

        // Compute the innovation, which is the difference between the measurement and the predicted state in measurement space
        Eigen::Matrix<float, MeasurementSize, 1> innovation = measurementVector - (measurementMatrix * stateVector);

        // std::cout << "Kalman::update - After Innovation" << std::endl;
        // std::cout << "innovation:\n" << innovation << std::endl;

        // Calculate the covariance of the innovation
        Eigen::Matrix<float, MeasurementSize, MeasurementSize> innovationCovariance = measurementMatrix * stateCovarianceMatrix * measurementMatrix.transpose() + measurementNoiseCovarianceMatrix;

        // std::cout << "Kalman::update - After Innovation Covariance" << std::endl;
        // std::cout << "innovationCovariance:\n" << innovationCovariance << std::endl;

        // Calculate the Kalman gain, which determines how much to correct the state based on the measurement
        Eigen::Matrix<float, StateSize, MeasurementSize> kalmanGain = stateCovarianceMatrix * measurementMatrix.transpose() * innovationCovariance.inverse();

        // std::cout << "Kalman::update - After Kalman Gain" << std::endl;
        // std::cout << "kalmanGain:\n" << kalmanGain << std::endl;

        // Update the state estimate using the innovation and the Kalman gain
        stateVector = stateVector + (kalmanGain * innovation);

        // std::cout << "Kalman::update - After State Vector Update" << std::endl;
        // std::cout << "stateVector:\n" << stateVector << std::endl;

        // Update the state covariance matrix using the Kalman gain
        stateCovarianceMatrix = (Eigen::Matrix<float, StateSize, StateSize>::Identity() - kalmanGain * measurementMatrix) * stateCovarianceMatrix;

        // std::cout << "Kalman::update - After State Covariance Update" << std::endl;
        // std::cout << "stateCovarianceMatrix:\n" << stateCovarianceMatrix << std::endl;
    }

    /**
     * @brief Extended Kalman Filter update for nonlinear measurement models.
     *
     * @param h      Nonlinear measurement function: z = h(x)
     * @param H_jac  Measurement Jacobian: ∂h/∂x at current x
     * @param z      Actual measurement
     */
    void updateEKF(
        const std::function<Eigen::Matrix<float, MeasurementSize, 1>(const Eigen::Matrix<float, StateSize, 1> &)> &h,
        const Eigen::Matrix<float, MeasurementSize, StateSize> &H_jac,
        const Eigen::Matrix<float, MeasurementSize, 1> &z)
    {
        // std::cout << "Kalman::EKF update - Begin\n";
        // std::cout << "stateVector:\n" << stateVector << std::endl;

        // Nonlinear measurement prediction
        Eigen::Matrix<float, MeasurementSize, 1> z_pred = h(stateVector);

        // Innovation
        Eigen::Matrix<float, MeasurementSize, 1> innovation = z - z_pred;
        // std::cout << "innovation:\n" << innovation << std::endl;

        // Innovation covariance
        Eigen::Matrix<float, MeasurementSize, MeasurementSize> S = H_jac * stateCovarianceMatrix * H_jac.transpose() + measurementNoiseCovarianceMatrix;
        // std::cout << "innovationCovariance:\n" << S << std::endl;

        // Kalman gain
        Eigen::Matrix<float, StateSize, MeasurementSize> K = stateCovarianceMatrix * H_jac.transpose() * S.inverse();
        // std::cout << "kalmanGain:\n" << K << std::endl;

        // State update
        stateVector = stateVector + K * innovation;
        // std::cout << "updatedStateVector:\n" << stateVector << std::endl;

        // Covariance update
        Eigen::Matrix<float, StateSize, StateSize> I = Eigen::Matrix<float, StateSize, StateSize>::Identity();
        stateCovarianceMatrix = (I - K * H_jac) * stateCovarianceMatrix;

        // std::cout << "updatedStateCovariance:\n" << stateCovarianceMatrix << std::endl;
    }

    // Method to get the state estimate
    /**
     * @brief Retrieves the current estimate of the state vector.
     *
     * @return The current state vector.
     */
    Eigen::Matrix<float, StateSize, 1> getState() const
    {
        return stateVector;
    }
};