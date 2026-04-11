#include "gps/types.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <string_view>

namespace gps {

double distance_between(double lat1, double lon1,
                        double lat2, double lon2) noexcept
{
    constexpr double DEG2RAD = std::numbers::pi / 180.0;
    constexpr double R       = 6'371'009.0;

    const double dlon  = (lon2 - lon1) * DEG2RAD;
    const double sdlon = std::sin(dlon), cdlon = std::cos(dlon);
    lat1 *= DEG2RAD; lat2 *= DEG2RAD;
    const double sl1 = std::sin(lat1), cl1 = std::cos(lat1);
    const double sl2 = std::sin(lat2), cl2 = std::cos(lat2);
    double d = (cl1 * sl2) - (sl1 * cl2 * cdlon);
    d = d * d + (cl2 * sdlon) * (cl2 * sdlon);
    return std::atan2(std::sqrt(d), (sl1 * sl2) + (cl1 * cl2 * cdlon)) * R;
}

double course_to(double lat1, double lon1,
                 double lat2, double lon2) noexcept
{
    constexpr double DEG2RAD = std::numbers::pi / 180.0;
    constexpr double RAD2DEG = 180.0 / std::numbers::pi;

    const double dlon = (lon2 - lon1) * DEG2RAD;
    lat1 *= DEG2RAD; lat2 *= DEG2RAD;
    const double a = std::sin(dlon) * std::cos(lat2);
    const double b = std::cos(lat1) * std::sin(lat2) -
                     std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
    double bearing = std::atan2(a, b) * RAD2DEG;
    if (bearing < 0.0) bearing += 360.0;
    return bearing;
}

std::string_view cardinal(float course) noexcept
{
    static constexpr std::array<std::string_view, 16> dirs = {
        "N","NNE","NE","ENE","E","ESE","SE","SSE",
        "S","SSW","SW","WSW","W","WNW","NW","NNW"
    };
    return dirs[static_cast<int>((course + 11.25f) / 22.5f) % 16];
}

std::string_view fix_label(const Coordinate& c) noexcept
{
    switch (c.carr_soln) {
    case CarrierSolution::FixedRTK: return "RTK";
    case CarrierSolution::FloatRTK: return "FRTK";
    default: break;
    }
    switch (c.fix_type) {
    case FixType::Fix3D:
    case FixType::GnssDR:  return "3D";
    case FixType::Fix2D:   return "2D";
    case FixType::DR:      return "DR";
    default:               return "---";
    }
}

} // namespace gps
