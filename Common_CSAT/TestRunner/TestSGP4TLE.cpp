#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "sgp4_tle.hpp"

TEST_CASE("Valid TLE Parsing from Strings")
{
    std::string line1 = "1 25544U 98067A   23286.48767130  .00006978  00000-0  18535-4 0  9993";
    std::string line2 = "2 25544  51.6426  64.2311 0003747  94.4615 265.6960 15.49348970421734";

    std::optional<SGP4TwoLineElement> tle = sgp4_utils::parseTLE(line1, line2);
    CHECK(tle.has_value());
    CHECK(tle.value().satelliteNumber == 25544);
    CHECK(tle.value().epochYear == 23);
    CHECK(tle.value().epochDay == doctest::Approx(286.48767130f));
    CHECK(tle.value().meanMotionDerivative1 == doctest::Approx(0.00006978f));
    CHECK(tle.value().meanMotionDerivative2 == doctest::Approx(0.0f));
    CHECK(tle.value().bStarDrag == doctest::Approx(0.000018535f));
    CHECK(tle.value().ephemerisType == 0);
    CHECK(tle.value().elementNumber == 999);
    CHECK(tle.value().inclination == doctest::Approx(51.6426f));
    CHECK(tle.value().rightAscensionAscendingNode == doctest::Approx(64.2311f));
    CHECK(tle.value().eccentricity == doctest::Approx(0.0003747f));
    CHECK(tle.value().argumentOfPerigee == doctest::Approx(94.4615f));
    CHECK(tle.value().meanAnomaly == doctest::Approx(265.6960f));
    CHECK(tle.value().meanMotion == doctest::Approx(15.49348970f));
    CHECK(tle.value().revolutionNumberAtEpoch == 42173);
}

TEST_CASE("Valid TLE Parsing from Strings: ISS 06222025")
{
    std::string line1 = "1 25544U 98067A   25173.70435133  .00010306  00000-0  18707-3 0  9990";
    std::string line2 = "2 25544  51.6391 279.7295 0002026 272.7719 232.5001 15.50190580516013";

    std::optional<SGP4TwoLineElement> tle = sgp4_utils::parseTLE(line1, line2);
    CHECK(tle.has_value());
    CHECK(tle.value().satelliteNumber == 25544);
    CHECK(tle.value().epochYear == 25);
    CHECK(tle.value().epochDay == doctest::Approx(173.704));
    CHECK(tle.value().meanMotionDerivative1 == doctest::Approx(0.00010306f));
    CHECK(tle.value().meanMotionDerivative2 == doctest::Approx(0.0f));
    CHECK(tle.value().bStarDrag == doctest::Approx(0.00018707f));
    CHECK(tle.value().ephemerisType == 0);
    CHECK(tle.value().elementNumber == 999);
    CHECK(tle.value().inclination == doctest::Approx(51.6391f));
    CHECK(tle.value().rightAscensionAscendingNode == doctest::Approx(279.729f));
    CHECK(tle.value().eccentricity == doctest::Approx(0.0002026f));
    CHECK(tle.value().argumentOfPerigee == doctest::Approx(272.772f));
    CHECK(tle.value().meanAnomaly == doctest::Approx(232.5f));
    CHECK(tle.value().meanMotion == doctest::Approx(15.5019f));
    CHECK(tle.value().revolutionNumberAtEpoch == 51601);
}

TEST_CASE("Checksum Line 1")
{
    std::string line1 = "1 25544U 98067A   23286.48767130  .00006978  00000-0  18535-4 0  9990";
    std::string line2 = "2 25544  51.6426  64.2311 0003747  94.4615 265.6960 15.49348970421734";

    REQUIRE(! sgp4_utils::parseTLE(line1, line2).has_value());
}

TEST_CASE("Checksum Line 1")
{
    std::string line1 = "1 25544U 98067A   23286.48767130  .00006978  00000-0  18535-4 0  9994";
    std::string line2 = "2 25544  51.6426  64.2311 0003747  94.4615 265.6960 15.49348970421730";

    REQUIRE(! sgp4_utils::parseTLE(line1, line2).has_value());
}

TEST_CASE("Invalid TLE Length from String Constructor")
{
    std::string line1 = "1 25544U 98067A   23286.48767130  .00006978  00000-0  18535-4 0  999"; // Short line
    std::string line2 = "2 25544  51.6426  64.2311 0003747  94.4615 265.6960 15.49348970421734";

    REQUIRE(! sgp4_utils::parseTLE(line1, line2).has_value());
}

TEST_CASE("Full Constructor Test")
{
    int32_t satNum = 25544;
    uint8_t epochYr = 23;
    float epochDy = 286.48767130f;
    float mmd1 = 0.00006978f;
    float mmd2 = 0.0f;
    float bStar = 0.000018535f;
    uint8_t ephType = 0;
    uint16_t elemNum = 9994;
    float incl = 51.6426f;
    float raan = 64.2311f;
    float ecc = 0.0003747f;
    float argp = 94.4615f;
    float mAnom = 265.6960f;
    float mMot = 15.49348970f;
    int32_t revNum = 421737;

    SGP4TwoLineElement tle{
        .satelliteNumber = satNum,
        .elementNumber = elemNum,
        .ephemerisType = ephType,
        .epochYear = epochYr,
        .epochDay = epochDy,
        .meanMotionDerivative1 = mmd1,
        .meanMotionDerivative2 = mmd2,
        .bStarDrag = bStar,
        .inclination = incl,
        .rightAscensionAscendingNode = raan,
        .eccentricity = ecc,
        .argumentOfPerigee = argp,
        .meanAnomaly = mAnom,
        .meanMotion = mMot,
        .revolutionNumberAtEpoch = revNum};

    CHECK(tle.satelliteNumber == satNum);
    CHECK(tle.epochYear == epochYr);
    CHECK(tle.epochDay == doctest::Approx(epochDy));
    CHECK(tle.meanMotionDerivative1 == doctest::Approx(mmd1));
    CHECK(tle.meanMotionDerivative2 == doctest::Approx(mmd2));
    CHECK(tle.bStarDrag == doctest::Approx(bStar));
    CHECK(tle.ephemerisType == ephType);
    CHECK(tle.elementNumber == elemNum);
    CHECK(tle.inclination == doctest::Approx(incl));
    CHECK(tle.rightAscensionAscendingNode == doctest::Approx(raan));
    CHECK(tle.eccentricity == doctest::Approx(ecc));
    CHECK(tle.argumentOfPerigee == doctest::Approx(argp));
    CHECK(tle.meanAnomaly == doctest::Approx(mAnom));
    CHECK(tle.meanMotion == doctest::Approx(mMot));
    CHECK(tle.revolutionNumberAtEpoch == revNum);
}

TEST_CASE("Padding")
{
    CHECK(sizeof(SGP4TwoLineElement) == 13 * 4);
}
