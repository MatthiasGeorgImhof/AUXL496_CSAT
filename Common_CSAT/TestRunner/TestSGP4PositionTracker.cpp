#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "SGP4PositionTracker.hpp"
#include <iostream>
#include <cmath>

TEST_CASE("Sgp4PositionTracker fuses intermittent GPS updates") {
    Sgp4PositionTracker tracker;
    float omega = 0.5f;

    for (int step = 0; step < 10; ++step) {
        float t = static_cast<float>(step + 1);

        Eigen::Vector3f pos(std::sin(omega * t),
                            std::cos(omega * t),
                            std::sin(omega * t) * std::cos(omega * t));
        Eigen::Vector3f vel(omega * std::cos(omega * t),
                           -omega * std::sin(omega * t),
                            omega * std::cos(2.0f * omega * t));

        tracker.setPrediction(pos, vel);

        // std::cout << "Step " << step + 1 << " — SGP4 pos: " << pos.transpose();

        if (step % 2 == 0) {
            Eigen::Vector3f gps_noise;
            gps_noise << static_cast<float>(rand() % 100 - 50) / 1000.f,
                         static_cast<float>(rand() % 100 - 50) / 1000.f,
                         static_cast<float>(rand() % 100 - 50) / 1000.f;
            Eigen::Vector3f gps = pos + gps_noise;

            tracker.updateWithGps(gps);
            // std::cout << " | GPS: " << gps.transpose();
        } else {
            // std::cout << " | GPS: ---";
        }

        auto est = tracker.getState();
        // std::cout << " | Fused pos: " << est.head<3>().transpose() << " | vel: " << est.tail<3>().transpose() << "\n";

        if (step % 2 == 0) {
            CHECK((est.head<3>() - pos).norm() < 0.1f);
            CHECK((est.tail<3>() - vel).norm() < 0.2f);
        }
    }
}
