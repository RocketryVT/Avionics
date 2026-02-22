#include "gps/GPSParser.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>


namespace gps {

GPSParser::GPSParser() {
  state_ = State::WAITING_FOR_DOLLAR;
  currentChecksum_ = 0;
  parsedChecksum_ = 0;

  currentCoordinate_.latitude = 0.0f;
  currentCoordinate_.longitude = 0.0f;
  currentCoordinate_.altitude = 0.0f;
  currentCoordinate_.satellites = 0;
  currentCoordinate_.valid = false;
}

void GPSParser::parse(char c) {
  switch (state_) {
  case State::WAITING_FOR_DOLLAR:
    if (c == '$') {
      state_ = State::READING_TYPE;
      buffer_.clear(); // Clear the string for new data
      currentChecksum_ = 0;
    }
    break;

  case State::READING_TYPE:
    if (c == ',') {
      currentChecksum_ ^= c;
      state_ = State::READING_DATA;
      buffer_ += ','; // Keep comma as separator
    } else if (c == '*') {
      state_ = State::READING_CHECKSUM_1;
    } else {
      buffer_ += c; // Add char to string
      currentChecksum_ ^= c;
    }
    break;

  case State::READING_DATA:
    if (c == '*') {
      state_ = State::READING_CHECKSUM_1;
    } else {
      buffer_ += c;
      currentChecksum_ ^= c;
    }
    break;

  case State::READING_CHECKSUM_1:
    if (c >= '0' && c <= '9')
      parsedChecksum_ = (c - '0');
    else
      parsedChecksum_ = (c - 'A' + 10);

    parsedChecksum_ = parsedChecksum_ * 16;
    state_ = State::READING_CHECKSUM_2;
    break;

  case State::READING_CHECKSUM_2:
    if (c >= '0' && c <= '9')
      parsedChecksum_ += (c - '0');
    else
      parsedChecksum_ += (c - 'A' + 10);

    if (parsedChecksum_ == currentChecksum_) {
      processSentence();
    }
    state_ = State::WAITING_FOR_DOLLAR;
    break;
  }
}

// Helper to find comma positions in std::string
// Returns the index AFTER the comma, or string::npos
static size_t findNextToken(const std::string &str, size_t start) {
  size_t pos = str.find(',', start);
  if (pos == std::string::npos)
    return std::string::npos;
  return pos + 1;
}

void GPSParser::processSentence() {
  // string.substr(0, 5) gives the first 5 chars
  if (buffer_.substr(0, 5) == "GPGGA") {
    parseGPGGA();
  } else if (buffer_.substr(0, 5) == "GPRMC") {
    parseGPRMC();
  }
}

void GPSParser::parseGPGGA() {
  // buffer_ looks like "GPGGA,123456,..."

  size_t pos = findNextToken(buffer_, 0); // Skip GPGGA
  if (pos == std::string::npos)
    return;

  pos = findNextToken(buffer_, pos); // Skip Time
  if (pos == std::string::npos)
    return;

  // Get Latitude
  // strtof works better with C-strings, so we use &buffer_[pos]
  float latRaw = strtof(&buffer_[pos], nullptr);
  pos = findNextToken(buffer_, pos);

  char ns = buffer_[pos]; // Read N or S directly
  pos = findNextToken(buffer_, pos);

  float lonRaw = strtof(&buffer_[pos], nullptr);
  pos = findNextToken(buffer_, pos);

  char ew = buffer_[pos];
  pos = findNextToken(buffer_, pos);

  int fixQuality = atoi(&buffer_[pos]);
  pos = findNextToken(buffer_, pos);

  int satellites = atoi(&buffer_[pos]);
  pos = findNextToken(buffer_, pos);

  pos = findNextToken(buffer_, pos); // Skip HDOP

  float altitude = strtof(&buffer_[pos], nullptr);

  if (fixQuality > 0) {
    int latDeg = (int)(latRaw / 100);
    float latMin = latRaw - (latDeg * 100);
    currentCoordinate_.latitude = latDeg + (latMin / 60.0f);
    if (ns == 'S')
      currentCoordinate_.latitude *= -1;

    int lonDeg = (int)(lonRaw / 100);
    float lonMin = lonRaw - (lonDeg * 100);
    currentCoordinate_.longitude = lonDeg + (lonMin / 60.0f);
    if (ew == 'W')
      currentCoordinate_.longitude *= -1;

    currentCoordinate_.altitude = altitude;
    currentCoordinate_.satellites = satellites;
    currentCoordinate_.valid = true;
  } else {
    currentCoordinate_.valid = false;
  }
}

void GPSParser::parseGPRMC() {
  size_t pos = findNextToken(buffer_, 0);
  if (pos == std::string::npos)
    return;

  pos = findNextToken(buffer_, pos); // Skip Time

  char status = buffer_[pos];
  pos = findNextToken(buffer_, pos);

  if (status != 'A') {
    currentCoordinate_.valid = false;
    return;
  }

  float latRaw = strtof(&buffer_[pos], nullptr);
  pos = findNextToken(buffer_, pos);

  char ns = buffer_[pos];
  pos = findNextToken(buffer_, pos);

  float lonRaw = strtof(&buffer_[pos], nullptr);
  pos = findNextToken(buffer_, pos);

  char ew = buffer_[pos];

  int latDeg = (int)(latRaw / 100);
  float latMin = latRaw - (latDeg * 100);
  currentCoordinate_.latitude = latDeg + (latMin / 60.0f);
  if (ns == 'S')
    currentCoordinate_.latitude *= -1;

  int lonDeg = (int)(lonRaw / 100);
  float lonMin = lonRaw - (lonDeg * 100);
  currentCoordinate_.longitude = lonDeg + (lonMin / 60.0f);
  if (ew == 'W')
    currentCoordinate_.longitude *= -1;

  currentCoordinate_.valid = true;
}

const Coordinate &GPSParser::getCoordinate() const {
  return currentCoordinate_;
}

bool GPSParser::hasFix() const { return currentCoordinate_.valid; }

} // namespace gps
