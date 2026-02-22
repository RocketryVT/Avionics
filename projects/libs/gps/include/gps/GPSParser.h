#pragma once

#include <cstdint>
#include <string> // using std::string is easier!

namespace gps {

struct Coordinate {
  float latitude;
  float longitude;
  float altitude;
  int satellites;
  bool valid;
};

class GPSParser {
public:
  GPSParser();

  void parse(char c);

  const Coordinate &getCoordinate() const;

  bool hasFix() const;

private:
  enum class State {
    WAITING_FOR_DOLLAR,
    READING_TYPE,
    READING_DATA,
    READING_CHECKSUM_1,
    READING_CHECKSUM_2
  };

  State state_;

  // Using string is much easier than managing char arrays!
  std::string buffer_;

  int currentChecksum_;
  int parsedChecksum_;

  Coordinate currentCoordinate_;

  void processSentence();
  void parseGPGGA();
  void parseGPRMC();
};

} // namespace gps
