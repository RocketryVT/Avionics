#pragma once

#include <cmath>

namespace rocket_math {

struct Location {
  double latitude;
  double longitude;
  double altitude; // Meters
};

class GroundStationMath {
public:
  static constexpr double EARTH_RADIUS_M = 6371000.0;

  // Calculates distance between two points (ignoring altitude)
  static double haversineDistance(const Location &loc1, const Location &loc2);

  // Calculates compass bearing (0-360 degrees)
  static double calculateAzimuth(const Location &loc1, const Location &loc2);

  // Calculates elevation angle (0-90 degrees)
  static double calculateElevation(const Location &loc1, const Location &loc2);

private:
  static double toRadians(double degrees);
  static double toDegrees(double radians);
};

} // namespace rocket_math
