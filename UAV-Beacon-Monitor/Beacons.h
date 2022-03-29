#ifndef Beacons_h
#define Beacons_h
#include "Arduino.h"
#include <time.h>

/**
    Enumeration des types de données reçues
*/
enum DATA_TYPE {
  RESERVED = 0,
  PROTOCOL_VERSION = 1,
  ID_FR = 2,
  ID_ANSI_CTA = 3,
  LATITUDE = 4,        // In WS84 in degree * 1e5
  LONGITUDE = 5,       // In WS84 in degree * 1e5
  ALTITUDE = 6,        // In MSL in m
  HEIGTH = 7,          // From Home in m
  HOME_LATITUDE = 8,   // In WS84 in degree * 1e5
  HOME_LONGITUDE = 9,  // In WS84 in degree * 1e5
  GROUND_SPEED = 10,   // In m/s
  HEADING = 11,        // Heading in degree from north 0 to 359.
  NOT_DEFINED_END = 12,
};

/**
    Tableau TLV (TYPE, LENGTH, VALUE) avec les tailles attendu des différents type données.
***/
static uint8_t TLV_LENGTH[] = {
  0,  // [DATA_TYPE::RESERVED]
  1,  // [DATA_TYPE::PROTOCOL_VERSION]
  30, // [DATA_TYPE::ID_FR]
  0,  // [DATA_TYPE::ID_ANSI_CTA]
  4,  // [DATA_TYPE::LATITUDE]
  4,  // [DATA_TYPE::LONGITUDE]
  2,  // [DATA_TYPE::ALTITUDE]
  2,  // [DATA_TYPE::HEIGTH]
  4,  // [DATA_TYPE::HOME_LATITUDE]
  4,  // [DATA_TYPE::HOME_LONGITUDE]
  1,  // [DATA_TYPE::GROUND_SPEED]
  2,  // [DATA_TYPE::HEADING]
};

typedef struct {
  bool use_ansi;
  uint8_t version;
  char id_fr[31];
  char id_ansi[21];
  int32_t lat;
  int32_t lon;
  int16_t alt;
  int16_t height;
  int32_t lat_start;
  int32_t lon_start;
  uint8_t speed;
  uint16_t bearing;
  time_t lastreceived_timestamp;
  time_t firstreceived_timestamp;
} drone_beacon_t;


uint8_t unpack_payload(uint8_t payload[], uint8_t len, drone_beacon_t *temp_beacon);
int32_t getCoordinates(uint8_t* data);
int16_t getAltitude(uint8_t* data);


#endif
