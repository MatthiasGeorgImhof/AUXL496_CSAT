#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN // Only define in one cpp file
#include "doctest.h"
#include "mock_hal.h"
#include "GNSS.hpp"
#include <iostream>
#include <vector>
#include <cstring>

TEST_CASE("GNSS GetNavPosECEF Trivial Test")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    clear_uart_rx_buffer();
    GNSS gnss(&huart);

    SUBCASE("Valid Data")
    {
        uint8_t test_data[] = {0xB5, 0x62, 0x01, 0x01, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xe7};
        static_assert(sizeof(test_data) == 28);
        inject_uart_rx_data(test_data, sizeof(test_data));

        std::optional<PositionECEF> navPosECEF = gnss.GetNavPosECEF();

        REQUIRE(navPosECEF.has_value());
        CHECK(navPosECEF.value().ecefX == 0);
        CHECK(navPosECEF.value().ecefY == 0);
        CHECK(navPosECEF.value().ecefZ == 0);
        CHECK(navPosECEF.value().pAcc == 0);
    }

    SUBCASE("Invalid Checksum")
    {
        uint8_t test_data[] = {0xB5, 0x62, 0x01, 0x01, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff};
        inject_uart_rx_data(test_data, sizeof(test_data));
        std::optional<PositionECEF> navPosECEF = gnss.GetNavPosECEF();
        CHECK(!navPosECEF.has_value());
    }

    SUBCASE("Wrong ClassID")
    {
        uint8_t test_data[] = {0xB5, 0x62, 0x02, 0x01, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Wrong ClassID
        inject_uart_rx_data(test_data, sizeof(test_data));
        std::optional<PositionECEF> navPosECEF = gnss.GetNavPosECEF();
        CHECK(!navPosECEF.has_value());
    }

    SUBCASE("Truncated Data")
    {
        uint8_t test_data[] = {0xB5, 0x62, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00}; // Truncated Data
        inject_uart_rx_data(test_data, sizeof(test_data));
        std::optional<PositionECEF> navPosECEF = gnss.GetNavPosECEF();
        CHECK(!navPosECEF.has_value());
    }
}

TEST_CASE("GNSS GetNavVelNED Trivial Test")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    clear_uart_rx_buffer();
    GNSS gnss(&huart);

    SUBCASE("Valid Data")
    {
        uint8_t test_data[] = {0xB5, 0x62, 0x01, 0x12, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x3e};
        static_assert(sizeof(test_data) == 44);
        inject_uart_rx_data(test_data, sizeof(test_data));

        std::optional<VelocityNED> navVelNED = gnss.GetNavVelNED();

        REQUIRE(navVelNED.has_value());
        CHECK(navVelNED.value().velN == 0);
        CHECK(navVelNED.value().velE == 0);
        CHECK(navVelNED.value().velD == 0);
        CHECK(navVelNED.value().headMot == 0);
        CHECK(navVelNED.value().gSpeed == 0);
        CHECK(navVelNED.value().sAcc == 0);
        CHECK(navVelNED.value().headAcc == 0);
    }
}

TEST_CASE("GNSS GetNavVelECEF Trivial Test")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    clear_uart_rx_buffer();
    GNSS gnss(&huart);

    SUBCASE("Valid Data")
    {
        uint8_t test_data[] = {0xB5, 0x62, 0x01, 0x11, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x57};
        static_assert(sizeof(test_data) == 28);
        inject_uart_rx_data(test_data, sizeof(test_data));

        std::optional<VelocityECEF> navVelECEF = gnss.GetNavVelECEF();

        REQUIRE(navVelECEF.has_value());
        CHECK(navVelECEF.value().ecefVX == 0);
        CHECK(navVelECEF.value().ecefVY == 0);
        CHECK(navVelECEF.value().ecefVZ == 0);
        CHECK(navVelECEF.value().sAcc == 0);
    }
}

TEST_CASE("GNSS getUniqID Test")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    clear_uart_rx_buffer();
    GNSS gnss(&huart);

    constexpr uint8_t test_data[] = {0xB5, 0x62, 0x27, 0x03, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 73, 249};
    static_assert(sizeof(test_data) == 18);
    static_assert(ValidateChecksum(test_data));
    inject_uart_rx_data(const_cast<uint8_t *>(test_data), sizeof(test_data));

    std::optional<UniqueID> uniqueID = gnss.getUniqID();

    REQUIRE(uniqueID.has_value());
    CHECK(uniqueID.value().id[0] == 0x01);
    CHECK(uniqueID.value().id[1] == 0x02);
    CHECK(uniqueID.value().id[2] == 0x03);
    CHECK(uniqueID.value().id[3] == 0x04);
    CHECK(uniqueID.value().id[4] == 0x05);
    CHECK(uniqueID.value().id[5] == 0x06);
}

TEST_CASE("GNSS GetNavPVT Test")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    clear_uart_rx_buffer();
    GNSS gnss(&huart);

    constexpr uint8_t test_data[] = {0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00, 0xA0, 0x3B, 0x56, 0x0F,
                                     0xE9, 0x07, 0x01, 0x07, 0x17, 0x1C, 0x0B, 0x37, 0x1F, 0x00,
                                     0x00, 0x00, 0x8B, 0x1C, 0xBD, 0x23, 0x03, 0x01, 0xEA, 0x05,
                                     0xE5, 0xF7, 0xED, 0xC6, 0x57, 0x4F, 0xB8, 0x11, 0xA2, 0x27,
                                     0x00, 0x00, 0x6E, 0x86, 0x00, 0x00, 0x20, 0x13, 0x00, 0x00,
                                     0x82, 0x12, 0x00, 0x00, 0xFF, 0xFD, 0xFF, 0xFF, 0xEF, 0x00,
                                     0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x36, 0x02, 0x00, 0x00,
                                     0x9E, 0x5C, 0xCD, 0x00, 0x3B, 0x09, 0x00, 0x00, 0xCD, 0xE0,
                                     0x2E, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 234, 192};
    static_assert(sizeof(test_data) == 100);
    static_assert(ValidateChecksum(test_data));
    inject_uart_rx_data(const_cast<uint8_t *>(test_data), sizeof(test_data));

    std::optional<NavigationPVT> navPVT = gnss.GetNavPVT();

    REQUIRE(navPVT.has_value());
    CHECK(navPVT.value().utcTime.year == 2025);
    CHECK(navPVT.value().utcTime.month == 1);
    CHECK(navPVT.value().utcTime.day == 7);
    CHECK(navPVT.value().utcTime.hour == 23);
    CHECK(navPVT.value().utcTime.min == 28);
    CHECK(navPVT.value().utcTime.sec == 11);
    CHECK(navPVT.value().utcTime.valid == 7);
    CHECK(navPVT.value().fixType == 3);

    CHECK(navPVT.value().position.lon == -957483035);
    CHECK(navPVT.value().position.lat == 297291607);
    CHECK(navPVT.value().position.height == 1014);
    CHECK(navPVT.value().position.hMSL == 3441);
    CHECK(navPVT.value().position.hAcc == 489);
    CHECK(navPVT.value().position.vAcc == 473);

    CHECK(navPVT.value().velocity.velN == -51);
    CHECK(navPVT.value().velocity.velE == 23);
    CHECK(navPVT.value().velocity.velD == 3);
    CHECK(navPVT.value().velocity.headMot == 13458590);
    CHECK(navPVT.value().velocity.speed == 0);
    CHECK(navPVT.value().velocity.gSpeed == 56);
    CHECK(navPVT.value().velocity.sAcc == 2363);
    CHECK(navPVT.value().velocity.headAcc == 3072205);
}

TEST_CASE("GNSS GetNavPosLLH Test")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    clear_uart_rx_buffer();
    GNSS gnss(&huart);

    constexpr uint8_t test_data[] = {0xB5, 0x62, 0x01, 0x02, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 58, 42};
    static_assert(sizeof(test_data) == 36);
    static_assert(ValidateChecksum(test_data));
    inject_uart_rx_data(const_cast<uint8_t *>(test_data), sizeof(test_data));

    std::optional<PositionLLH> navPos = gnss.GetNavPosLLH();

    REQUIRE(navPos.has_value());
    CHECK(navPos.value().lon == 26);
    CHECK(navPos.value().lat == 1);
    CHECK(navPos.value().height == 0);
    CHECK(navPos.value().hMSL == 0);
    CHECK(navPos.value().hAcc == 0);
    CHECK(navPos.value().vAcc == 0);
}

TEST_CASE("GNSS GetNavTimeUTC Test")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    clear_uart_rx_buffer();
    GNSS gnss(&huart);

    constexpr uint8_t test_data[] = {0xB5, 0x62, 0x01, 0x21, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x07, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 37, 56};
    static_assert(sizeof(test_data) == 28);
    static_assert(ValidateChecksum(test_data));
    inject_uart_rx_data(const_cast<uint8_t *>(test_data), sizeof(test_data));

    std::optional<UTCTime> utcTime = gnss.GetNavTimeUTC();

    REQUIRE(utcTime.has_value());
    CHECK(utcTime.value().year == 2024);
    CHECK(utcTime.value().month == 0);
    CHECK(utcTime.value().day == 0);
    CHECK(utcTime.value().hour == 0);
    CHECK(utcTime.value().min == 0);
    CHECK(utcTime.value().sec == 0);
}

#include "au.hpp"

TEST_CASE("PositionECEF_AU ConvertPositionECEF(const PositionECEF &pos")
{
    PositionECEF pos = {1000000, 2000000, 3000000, 500};
    PositionECEF_AU auPos = ConvertPositionECEF(pos);

    CHECK(auPos.x.in(au::meters * au::ecefs) == doctest::Approx(static_cast<float>(pos.ecefX)/100.f));
    CHECK(auPos.y.in(au::meters * au::ecefs) == doctest::Approx(static_cast<float>(pos.ecefY)/100.f));
    CHECK(auPos.z.in(au::meters * au::ecefs) == doctest::Approx(static_cast<float>(pos.ecefZ)/100.f));
    CHECK(auPos.acc.in(au::meters * au::ecefs) == doctest::Approx(static_cast<float>(pos.pAcc)/100.f));
}

TEST_CASE("PositionECEF_AU ConvertPositionECEF(const PositionECEF &neg")
{
    PositionECEF pos = {-1000000, -2000000, -3000000, 500};
    PositionECEF_AU auPos = ConvertPositionECEF(pos);

    CHECK(auPos.x.in(au::meters * au::ecefs) == doctest::Approx(static_cast<float>(pos.ecefX)/100.f));
    CHECK(auPos.y.in(au::meters * au::ecefs) == doctest::Approx(static_cast<float>(pos.ecefY)/100.f));
    CHECK(auPos.z.in(au::meters * au::ecefs) == doctest::Approx(static_cast<float>(pos.ecefZ)/100.f));
    CHECK(auPos.acc.in(au::meters * au::ecefs) == doctest::Approx(static_cast<float>(pos.pAcc)/100.f));
}

TEST_CASE("VelocityECEF_AU ConvertVelocityECEF(const VelocityECEF &vel")
{
    VelocityECEF vel = {1000000, 2000000, 3000000, 500};
    VelocityECEF_AU auVel = ConvertVelocityECEF(vel);

    CHECK(auVel.x.in(au::meters *  au::ecefs / au::seconds) == doctest::Approx(static_cast<float>(vel.ecefVX)/100.f));
    CHECK(auVel.y.in(au::meters *  au::ecefs / au::seconds) == doctest::Approx(static_cast<float>(vel.ecefVY)/100.f));
    CHECK(auVel.z.in(au::meters *  au::ecefs / au::seconds) == doctest::Approx(static_cast<float>(vel.ecefVZ)/100.f));
    CHECK(auVel.acc.in(au::meters *  au::ecefs / au::seconds) == doctest::Approx(static_cast<float>(vel.sAcc)/100.f));
}

TEST_CASE("VelocityECEF_AU ConvertVelocityECEF(const VelocityECEF &neg")
{
    VelocityECEF vel = {-1000000, -2000000, -3000000, 500};
    VelocityECEF_AU auVel = ConvertVelocityECEF(vel);

    CHECK(auVel.x.in(au::meters *  au::ecefs / au::seconds) == doctest::Approx(static_cast<float>(vel.ecefVX)/100.f));
    CHECK(auVel.y.in(au::meters *  au::ecefs / au::seconds) == doctest::Approx(static_cast<float>(vel.ecefVY)/100.f));
    CHECK(auVel.z.in(au::meters *  au::ecefs / au::seconds) == doctest::Approx(static_cast<float>(vel.ecefVZ)/100.f));
    CHECK(auVel.acc.in(au::meters *  au::ecefs / au::seconds) == doctest::Approx(static_cast<float>(vel.sAcc)/100.f));
}



