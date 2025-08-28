#pragma once

#include "au.hh"

namespace au
{
    typedef decltype(Meters{} / Seconds{}) MetersPerSecond;
    typedef decltype(Meters{} / (Seconds{} * Seconds{})) MetersPerSecondSquared;
    typedef decltype(Degrees{} / Seconds{}) DegreesPerSecond;
    typedef decltype(Degrees{} / (Seconds{} * Seconds{})) DegreesPerSecondSquared;
    typedef decltype(Radians{} / Seconds{}) RadiansPerSecond;
    typedef decltype(Radians{} / (Seconds{} * Seconds{})) RadiansPerSecondSquared;
    constexpr auto metersPerSecond = meters / second;
    constexpr auto metersPerSecondSquared = meters / second / second;
    constexpr auto degreesPerSecond = degrees / second;
    constexpr auto degreesPerSecondSquared = degrees / second / second;
    constexpr auto radiansPerSecond = radians / second;
    constexpr auto radiansPerSecondSquared = radians / second / second;


    struct BodyBaseDim : base_dim::BaseDimension<1690384950> {};
    struct Bodys : UnitImpl<Dimension<BodyBaseDim>>
    {
        static constexpr inline const char label[] = "body";
    };
    constexpr auto body = SingularNameFor<Bodys>{};
    constexpr auto bodys = QuantityMaker<Bodys>{};
    typedef decltype(Bodys{} * Meters{}) MetersInBodyFrame;
    typedef decltype(Bodys{} * MetersPerSecond{}) MetersPerSecondInBodyFrame;
    typedef decltype(Bodys{} * MetersPerSecondSquared{}) MetersPerSecondSquaredInBodyFrame;
    typedef decltype(Bodys{} * Degrees{}) DegreesInBodyFrame;
    typedef decltype(Bodys{} * DegreesPerSecond{}) DegreesPerSecondInBodyFrame;
    typedef decltype(Bodys{} * DegreesPerSecondSquared{}) DegreesPerSecondSquaredInBodyFrame;
    typedef decltype(Bodys{} * Radians{}) RadiansInBodyFrame;
    typedef decltype(Bodys{} * RadiansPerSecond{}) RadiansPerSecondInBodyFrame;
    typedef decltype(Bodys{} * RadiansPerSecondSquared{}) RadiansPerSecondSquaredInBodyFrame;
    typedef decltype(Bodys{} * Tesla{}) TeslaInBodyFrame;
    constexpr auto metersInBodyFrame = bodys * meters;
    constexpr auto metersPerSecondInBodyFrame = bodys * metersPerSecond;
    constexpr auto metersPerSecondSquaredInBodyFrame = bodys * metersPerSecondSquared;
    constexpr auto degreesInBodyFrame = bodys * degrees;
    constexpr auto degreesPerSecondInBodyFrame = bodys * degreesPerSecond;
    constexpr auto degreesPerSecondSquaredInBodyFrame = bodys * degreesPerSecondSquared;
    constexpr auto radiansInBodyFrame = bodys * radians;
    constexpr auto radiansPerSecondInBodyFrame = bodys * radiansPerSecond;
    constexpr auto radiansPerSecondSquaredInBodyFrame = bodys * radiansPerSecondSquared;
    constexpr auto teslaInBodyFrame = bodys * tesla;


    struct TemeBaseDim : base_dim::BaseDimension<1690384951> {};
    struct Temes : UnitImpl<Dimension<TemeBaseDim>>
    {
        static constexpr inline const char label[] = "teme";
    };
    constexpr auto teme = SingularNameFor<Temes>{};
    constexpr auto temes = QuantityMaker<Temes>{};
    typedef decltype(Temes{} * Meters{}) MetersInTemeFrame;
    typedef decltype(Temes{} * MetersPerSecond{}) MetersPerSecondInTemeFrame;
    typedef decltype(Temes{} * MetersPerSecondSquared{}) MetersPerSecondSquaredInTemeFrame;
    typedef decltype(Temes{} * Degrees{}) DegreesInTemeFrame;
    typedef decltype(Temes{} * DegreesPerSecond{}) DegreesPerSecondInTemeFrame;
    typedef decltype(Temes{} * DegreesPerSecondSquared{}) DegreesPerSecondSquaredInTemeFrame;
    typedef decltype(Temes{} * Radians{}) RadiansInTemeFrame;
    typedef decltype(Temes{} * RadiansPerSecond{}) RadiansPerSecondInTemeFrame;
    typedef decltype(Temes{} * RadiansPerSecondSquared{}) RadiansPerSecondSquaredInTemeFrame;
    typedef decltype(Temes{} * Tesla{}) TeslaInTemeFrame;
    constexpr auto metersInTemeFrame = temes * meters;
    constexpr auto metersPerSecondInTemeFrame = temes * metersPerSecond;
    constexpr auto metersPerSecondSquaredInTemeFrame = temes * metersPerSecondSquared;
    constexpr auto degreesInTemeFrame = temes * degrees;
    constexpr auto degreesPerSecondInTemeFrame = temes * degreesPerSecond;
    constexpr auto degreesPerSecondSquaredInTemeFrame = temes * degreesPerSecondSquared;
    constexpr auto radiansInTemeFrame = temes * radians;
    constexpr auto radiansPerSecondInTemeFrame = temes * radiansPerSecond;
    constexpr auto radiansPerSecondSquaredInTemeFrame = temes * radiansPerSecondSquared;
    constexpr auto teslaInTemeFrame = temes * tesla;

    
    struct NedBaseDim : base_dim::BaseDimension<1690384952> {};
    struct Neds : UnitImpl<Dimension<NedBaseDim>>
    {
        static constexpr inline const char label[] = "ned";
    };
    constexpr auto ned = SingularNameFor<Neds>{};
    constexpr auto neds = QuantityMaker<Neds>{};
    typedef decltype(Neds{} * Meters{}) MetersInNedFrame;
    typedef decltype(Neds{} * MetersPerSecond{}) MetersPerSecondInNedFrame;
    typedef decltype(Neds{} * MetersPerSecondSquared{}) MetersPerSecondSquaredInNedFrame;
    typedef decltype(Neds{} * Degrees{}) DegreesInNedFrame;
    typedef decltype(Neds{} * DegreesPerSecond{}) DegreesPerSecondInNedFrame;
    typedef decltype(Neds{} * DegreesPerSecondSquared{}) DegreesPerSecondSquaredInNedFrame;
    typedef decltype(Neds{} * Radians{}) RadiansInNedFrame;
    typedef decltype(Neds{} * RadiansPerSecond{}) RadiansPerSecondInNedFrame;
    typedef decltype(Neds{} * RadiansPerSecondSquared{}) RadiansPerSecondSquaredInNedFrame;
    typedef decltype(Neds{} * Tesla{}) TeslaInNedFrame;
    constexpr auto metersInNedFrame = neds * meters;
    constexpr auto metersPerSecondInNedFrame = neds * metersPerSecond;
    constexpr auto metersPerSecondSquaredInNedFrame = neds * metersPerSecondSquared;
    constexpr auto degreesInNedFrame = neds * degrees;
    constexpr auto degreesPerSecondInNedFrame = neds * degreesPerSecond;
    constexpr auto degreesPerSecondSquaredInNedFrame = neds * degreesPerSecondSquared;
    constexpr auto radiansInNedFrame = neds * radians;
    constexpr auto radiansPerSecondInNedFrame = neds * radiansPerSecond;
    constexpr auto radiansPerSecondSquaredInNedFrame = neds * radiansPerSecondSquared;
    constexpr auto teslaInNedFrame = neds * tesla;

    
    struct EcefBaseDim : base_dim::BaseDimension<1690384953> {};
    struct Ecefs : UnitImpl<Dimension<EcefBaseDim>>
    {
        static constexpr inline const char label[] = "ecef";
    };
    constexpr auto ecef = SingularNameFor<Ecefs>{};
    constexpr auto ecefs = QuantityMaker<Ecefs>{};
    typedef decltype(Ecefs{} * Meters{}) MetersInEcefFrame;
    typedef decltype(Ecefs{} * MetersPerSecond{}) MetersPerSecondInEcefFrame;
    typedef decltype(Ecefs{} * MetersPerSecondSquared{}) MetersPerSecondSquaredInEcefFrame;
    typedef decltype(Ecefs{} * Degrees{}) DegreesInEcefFrame;
    typedef decltype(Ecefs{} * DegreesPerSecond{}) DegreesPerSecondInEcefFrame;
    typedef decltype(Ecefs{} * DegreesPerSecondSquared{}) DegreesPerSecondSquaredInEcefFrame;
    typedef decltype(Ecefs{} * Radians{}) RadiansInEcefFrame;
    typedef decltype(Ecefs{} * RadiansPerSecond{}) RadiansPerSecondInEcefFrame;
    typedef decltype(Ecefs{} * RadiansPerSecondSquared{}) RadiansPerSecondSquaredInEcefFrame;
    typedef decltype(Ecefs{} * Tesla{}) TeslaInEcefFrame;
    constexpr auto metersInEcefFrame = ecefs * meters;
    constexpr auto metersPerSecondInEcefFrame = ecefs * metersPerSecond;
    constexpr auto metersPerSecondSquaredInEcefFrame = ecefs * metersPerSecondSquared;
    constexpr auto degreesInEcefFrame = ecefs * degrees;
    constexpr auto degreesPerSecondInEcefFrame = ecefs * degreesPerSecond;
    constexpr auto degreesPerSecondSquaredInEcefFrame = ecefs * degreesPerSecondSquared;
    constexpr auto radiansInEcefFrame = ecefs * radians;
    constexpr auto radiansPerSecondInEcefFrame = ecefs * radiansPerSecond;
    constexpr auto radiansPerSecondSquaredInEcefFrame = ecefs * radiansPerSecondSquared;
    constexpr auto teslaInEcefFrame = ecefs * tesla;


    struct GeodeticBaseDim : base_dim::BaseDimension<1690384954> {};
    struct Geodetics : UnitImpl<Dimension<GeodeticBaseDim>>
    {
        static constexpr inline const char label[] = "geodetic";
    };
    constexpr auto geodetic = SingularNameFor<Geodetics>{};
    constexpr auto geodetics = QuantityMaker<Geodetics>{};
    typedef decltype(Geodetics{} * Meters{}) MetersInGeodeticFrame;
    typedef decltype(Geodetics{} * MetersPerSecond{}) MetersPerSecondInGeodeticFrame;
    typedef decltype(Geodetics{} * MetersPerSecondSquared{}) MetersPerSecondSquaredInGeodeticFrame;
    typedef decltype(Geodetics{} * Degrees{}) DegreesInGeodeticFrame;
    typedef decltype(Geodetics{} * DegreesPerSecond{}) DegreesPerSecondInGeodeticFrame;
    typedef decltype(Geodetics{} * DegreesPerSecondSquared{}) DegreesPerSecondSquaredInGeodeticFrame;
    typedef decltype(Geodetics{} * Radians{}) RadiansInGeodeticFrame;
    typedef decltype(Geodetics{} * RadiansPerSecond{}) RadiansPerSecondInGeodeticFrame;
    typedef decltype(Geodetics{} * RadiansPerSecondSquared{}) RadiansPerSecondSquaredInGeodeticFrame;
    typedef decltype(Geodetics{} * Tesla{}) TeslaInGeodeticFrame;
    constexpr auto metersInGeodeticFrame = geodetics * meters;
    constexpr auto metersPerSecondInGeodeticFrame = geodetics * metersPerSecond;
    constexpr auto metersPerSecondSquaredInGeodeticFrame = geodetics * metersPerSecondSquared;
    constexpr auto degreesInGeodeticFrame = geodetics * degrees;
    constexpr auto degreesPerSecondInGeodeticFrame = geodetics * degreesPerSecond;
    constexpr auto degreesPerSecondSquaredInGeodeticFrame = geodetics * degreesPerSecondSquared;
    constexpr auto radiansInGeodeticFrame = geodetics * radians;
    constexpr auto radiansPerSecondInGeodeticFrame = geodetics * radiansPerSecond;
    constexpr auto radiansPerSecondSquaredInGeodeticFrame = geodetics * radiansPerSecondSquared;
    constexpr auto teslaInGeodeticFrame = geodetics * tesla;

    struct GeocentricBaseDim : base_dim::BaseDimension<1690384955> {};
    struct Geocentrics : UnitImpl<Dimension<GeocentricBaseDim>>
    {
        static constexpr inline const char label[] = "geocentric";
    };
    constexpr auto geocentric = SingularNameFor<Geocentrics>{};
    constexpr auto geocentrics = QuantityMaker<Geocentrics>{};
    typedef decltype(Geocentrics{} * Meters{}) MetersInGeocentricFrame;
    typedef decltype(Geocentrics{} * MetersPerSecond{}) MetersPerSecondInGeocentricFrame;
    typedef decltype(Geocentrics{} * MetersPerSecondSquared{}) MetersPerSecondSquaredInGeocentricFrame;
    typedef decltype(Geocentrics{} * Degrees{}) DegreesInGeocentricFrame;
    typedef decltype(Geocentrics{} * DegreesPerSecond{}) DegreesPerSecondInGeocentricFrame;
    typedef decltype(Geocentrics{} * DegreesPerSecondSquared{}) DegreesPerSecondSquaredInGeocentricFrame;
    typedef decltype(Geocentrics{} * Radians{}) RadiansInGeocentricFrame;
    typedef decltype(Geocentrics{} * RadiansPerSecond{}) RadiansPerSecondInGeocentricFrame;
    typedef decltype(Geocentrics{} * RadiansPerSecondSquared{}) RadiansPerSecondSquaredInGeocentricFrame;
    typedef decltype(Geocentrics{} * Tesla{}) TeslaInGeocentricFrame;
    constexpr auto metersInGeocentricFrame = geocentrics * meters;
    constexpr auto metersPerSecondInGeocentricFrame = geocentrics * metersPerSecond;
    constexpr auto metersPerSecondSquaredInGeocentricFrame = geocentrics * metersPerSecondSquared;
    constexpr auto degreesInGeocentricFrame = geocentrics * degrees;
    constexpr auto degreesPerSecondInGeocentricFrame = geocentrics * degreesPerSecond;
    constexpr auto degreesPerSecondSquaredInGeocentricFrame = geocentrics * degreesPerSecondSquared;
    constexpr auto radiansInGeocentricFrame = geocentrics * radians;
    constexpr auto radiansPerSecondInGeocentricFrame = geocentrics * radiansPerSecond;
    constexpr auto radiansPerSecondSquaredInGeocentricFrame = geocentrics * radiansPerSecondSquared;
    constexpr auto teslaInGeocentricFrame = geocentrics * tesla;


}
