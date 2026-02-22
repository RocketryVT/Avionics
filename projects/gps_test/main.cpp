#include "gps/GPSParser.h"
#include "math_utils/GroundStationMath.h"
#include <cstdio>
#include <iostream>

void testGPS() {
  printf("Testing GPS Parser...\n");
  gps::GPSParser parser;

  // Test GPGGA
  const char *gga =
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  for (int i = 0; gga[i] != '\0'; ++i) {
    parser.parse(gga[i]);
  }

  if (parser.hasFix()) {
    auto coord = parser.getCoordinate();
    printf("GPGGA Fix: Lat %.6f, Lon %.6f, Alt %.1f\n", coord.latitude,
           coord.longitude, coord.altitude);
    // Expected: 48.1173, 11.5166, 545.4
  } else {
    printf("GPGGA Fix Failed!\n");
  }

  // Test GPRMC
  const char *rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,"
                    "003.1,W*6A\r\n";
  for (int i = 0; rmc[i] != '\0'; ++i) {
    parser.parse(rmc[i]);
  }

  if (parser.hasFix()) {
    auto coord = parser.getCoordinate();
    printf("GPRMC Fix: Lat %.6f, Lon %.6f\n", coord.latitude, coord.longitude);
  } else {
    printf("GPRMC Fix Failed!\n");
  }
}

void testMath() {
  printf("\nTesting Math Utils...\n");

  // Drillfield
  rocket_math::Location p1 = {37.2278, -80.4222, 634.0};
  // Lane Stadium
  rocket_math::Location p2 = {37.2197, -80.4176, 634.0};

  double dist = rocket_math::GroundStationMath::haversineDistance(p1, p2);
  double az = rocket_math::GroundStationMath::calculateAzimuth(p1, p2);

  printf("Distance (Drillfield -> Lane): %.2f meters\n", dist);
  printf("Azimuth (Drillfield -> Lane): %.2f degrees\n", az);

  // Test Elevation
  rocket_math::Location ground = {0.0, 0.0, 0.0};
  rocket_math::Location rocket = {0.0, 0.01, 1000.0}; // ~1.1km away ground distance, 1km up
  double elev =
      rocket_math::GroundStationMath::calculateElevation(ground, rocket);
  printf("Elevation (1km up, ~1.1km away): %.2f degrees\n", elev);
}

int main() {
  testGPS();
  testMath();
  return 0;
}
