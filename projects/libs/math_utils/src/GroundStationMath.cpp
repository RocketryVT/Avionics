#include "math_utils/GroundStationMath.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace rocket_math {

double GroundStationMath::toRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

double GroundStationMath::toDegrees(double radians) {
  return radians * 180.0 / M_PI;
}

// Haversine Formula for spherical distance
double GroundStationMath::haversineDistance(const Location &loc1,
                                            const Location &loc2) {
  double lat1 = toRadians(loc1.latitude);
  double lon1 = toRadians(loc1.longitude);
  double lat2 = toRadians(loc2.latitude);
  double lon2 = toRadians(loc2.longitude);

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  // a = sin²(Δlat/2) + cos(lat1) * cos(lat2) * sin²(Δlon/2)
  double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
             std::cos(lat1) * std::cos(lat2) * std::sin(dLon / 2.0) *
                 std::sin(dLon / 2.0);

  double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

  return EARTH_RADIUS_M * c;
}

// Calculates compass bearing
double GroundStationMath::calculateAzimuth(const Location &loc1,
                                           const Location &loc2) {
  double lat1 = toRadians(loc1.latitude);
  double lon1 = toRadians(loc1.longitude);
  double lat2 = toRadians(loc2.latitude);
  double lon2 = toRadians(loc2.longitude);
  double dLon = lon2 - lon1;

  double y = std::sin(dLon) * std::cos(lat2);
  double x = std::cos(lat1) * std::sin(lat2) -
             std::sin(lat1) * std::cos(lat2) * std::cos(dLon);

  double bearing = std::atan2(y, x);
  double bearingDeg = toDegrees(bearing);

  // Normalize to 0-360
  return std::fmod(bearingDeg + 360.0, 360.0);
}

// Calculates elevation angle
double GroundStationMath::calculateElevation(const Location &loc1, const Location &loc2) {
  double groundDist = haversineDistance(loc1, loc2);
  double altDiff = loc2.altitude - loc1.altitude;

  // angle = atan(opposite / adjacent)
  return toDegrees(std::atan2(altDiff, groundDist));
}

} // namespace rocket_math
