#include "Beacons.h"

uint8_t unpack_payload(uint8_t payload[], uint8_t len, drone_beacon_t *temp_beacon) {
  /*
    Frame payload: 80,0,0,0,FF,FF,FF,FF,FF,FF,2C,25,B5,F3,45,E6
                ,2C,25,B5,F3,45,E6,C0,6C,C0,BE,EB,88,01,00,00
                ,00,64,00,01,04,00,
                0F,   // SSID LENGTH
                41,4D,53,20,54,52,41,4E,53,4D,49,54,54,45,52,  // SSID
                DD,       // Vendor specific marker
                4A,       // Length
                6A,5C,35, // OUI
                01,       // Type
                01,01,01, // Version
                02,1E,41,4D,53,32,30,42,30,30,30,30,30,30,30,30,30,30,30,30,30,30,32,30,32,31,30,31,31,31,32,32, // ID
                03,14,51,51,52,53,0F,41,41,41,41,41,41,42,43,44,45,46,47,48,49,4A, // Tag ANSI CTA 2063 UAS = [4 Character MFR CODE][1 Character LENGTH CODE][15 Characters MAX MANUFACTURER’S SERIAL NUMBER]
                04,04,00,42,8C,C6,  // Lat
                05,04,00,02,21,1E,  // Lon
                06,02,00,9B,        // Alt
                08,04,00,42,8C,CF,  // Lat dep
                09,04,00,02,21,1D,  // Lon dep
                0A,01,00,           // Speed
                0B,02,00,           // Route
                BC,01,08,82,84,8B,96,24,30,48,6C,3,1,6,0,0,78,56, // Else

        test packet:
        uint8_t temp_data[172] = {0x80,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                                   0x2C,0x25,0xB5,0xF3,0x45,0xE6,0x2C,0x25,0xB5,0xF3,
                                   0x45,0xE6,0xC0,0x6C,0xC0,0xBE,0xEB,0x88,0x01,0x00,
                                   0x00,0x00,0x64,0x00,0x01,0x04,0x00,0x0F,0x41,0x4D,
                                   0x53,0x20,0x54,0x52,0x41,0x4E,0x53,0x4D,0x49,0x54,
                                   0x54,0x45,0x52,
                                   0xDD,0x61,
                                   0x6A,0x5C,0x35,
                                   0x01,0x01,
                                   0x01,0x01,
                                   0x02,0x1E,0x41,0x4D,0x53,0x32,0x30,0x42,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,
                                   0x30,0x30,0x30,0x30,0x32,0x30,0x32,0x31,0x30,0x31,0x31,0x31,0x32,0x32,
                                   0x03,0x14,0x51,0x51,0x52,0x53,
                                   0x0F,0x41,0x41,0x41,0x41,0x41,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,
                                   0x04,0x04,0x00,0x42,0x8C,0xC6,
                                   0x05,0x04,0x00,0x02,0x21,0x1E,
                                   0x06,0x02,0x00,0x9B,
                                   0x07,0x02,0x00,0x9B,
                                   0x08,0x04,0x00,0x42,0x8C,0xCF,
                                   0x09,0x04,0x00,0x02,0x21,0x1D,
                                   0x0A,0x01,0x00,
                                   0x0B,0x02,0x00,0xBC,
                                   0x01,0x08,0x82,0x84,0x8B,0x96,0x24,0x30,0x48,0x6C,0x3,0x1,0x6,0x0,0x0,0x78,0x56};
  */
  uint8_t SSID_length = (int)payload[37];
  uint8_t offset_VS = 38 + SSID_length;

  // Check that VS is present
  if (payload[offset_VS] != 0xDD) {
    return 1;
  }

  // Get length and check OUI
  uint8_t vs_len = payload[offset_VS + 1];

  if (vs_len + 54 > len) {
    return 2;
  }
  //Filter OUI from 6A:5C:35
  if (payload[offset_VS + 2] != 0x6A || payload[offset_VS + 3] != 0x5C || payload[offset_VS + 4] != 0x35) {
    return 3;
  }

  // Check that VS Type is 1
  if (payload[offset_VS + 5] != 0x01) {
    return 4;
  }

  uint8_t offset = offset_VS + 6; // We move past 0xDD, OUI, VS_Type

  do {
    uint8_t type = payload[offset ++];
    uint8_t len_data = payload[offset ++];
    switch (type) {
      case 1: // Version
        if (len_data == 1) {
          temp_beacon->version = payload[offset];
        }
        break;
      case 2: // ID_FR
        if (len_data == 30) {
          // We reduce consecutive 0s to "::"
          uint8_t current_char = 0;
          for (uint8_t i = 0; i < len_data; i++) {
            if (payload[i + offset] == 0x30) {
              // look forward to find the end of the 0s
              uint8_t last0 = i;
              for (uint8_t j = i + 1; j < len_data; j++) {
                if (payload[j + offset] == 0x30) {
                  last0 = j;
                }
                else {
                  break;
                }
              }
              if (last0 == i) {
                temp_beacon->id_fr[current_char++] = payload[i + offset];
              }
              else {
                //Moar 0s!
                temp_beacon->id_fr[current_char++] = 0x3A; // ":" character
                temp_beacon->id_fr[current_char++] = 0x3A;
                i = last0;
              }
            }
            else {
              temp_beacon->id_fr[current_char++] = payload[i + offset];
            }
          }
          temp_beacon->id_fr[current_char] = 0;
          temp_beacon->use_ansi = false;
        }
        break;
      case 3: // ID_ANSI
        // Structure is SN = [4 Character MFR CODE][1 Character LENGTH CODE][15 Character MANUFACTURER’S SERIAL NUMBER]
        if (len_data <= 20) {
          uint8_t len_id = payload[offset + 4];
          if (len_id + 5 > len_data) {
            // Invalid ANSI ID, len_id is longer than allocated space
            break;
          }

          uint8_t current_char = 0;

          for (uint8_t i = 0; i < 4 + 1 + len_id; i++) {
            if ( i == 4 ) {
              temp_beacon->id_ansi[current_char] = 0x2D; // We replace the length in the ID by a dash
            }
            else {
              temp_beacon->id_ansi[current_char] = payload[offset + i];
            }
            current_char++;
          }

          temp_beacon->id_ansi[current_char] = 0;
          temp_beacon->use_ansi = true;
        }
        break;
      case 4: // Lat
        if (len_data == 4) {
          temp_beacon->lat = getCoordinates(&payload[offset]);
        }
        break;
      case 5: // Lon
        if (len_data == 4) {
          temp_beacon->lon = getCoordinates(&payload[offset]);
        }
        break;
      case 6: // Alt
        if (len_data == 2) {
          temp_beacon->alt = getAltitude(&payload[offset]);
        }
        break;
      case 7: // Height
        if (len_data == 2) {
          temp_beacon->height = getAltitude(&payload[offset]);
        }
        break;
      case 8: // Lat start
        if (len_data == 4) {
          temp_beacon->lat_start = getCoordinates(&payload[offset]);
        }
        break;
      case 9:  // Lon start
        if (len_data == 4) {
          temp_beacon->lon_start = getCoordinates(&payload[offset]);
        }
        break;
      case 10: // Speed
        if (len_data == 1) {
          temp_beacon->speed = payload[offset];
        }
        break;
      case 11: // Bearing
        if (len_data == 2) {
          temp_beacon->bearing = payload[offset] << 8 | payload[offset + 1];
        }
        break;
      default:
        break;
    }
    offset += len_data;
  } while ( offset < vs_len + offset_VS + 4 ); // Limit of VS frame is after OUI
  time_t now;
  time(&now);
  temp_beacon->lastreceived_timestamp = now;
  temp_beacon->firstreceived_timestamp = now;
  return 0;
}


int32_t getCoordinates(uint8_t* data) {
  bool neg_number = data[0] > 0x7F;
  int32_t data_value = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];

  if (neg_number) {
    data_value = (0xFFFFFFFF & ~data_value) + 1;
    data_value *= -1;
  }

  return data_value;
}

int16_t getAltitude(uint8_t* data) {
  bool neg_number = data[0] > 0x7F;

  int16_t data_value =  data[0] << 8 | data[1];

  if (neg_number) {
    data_value = (0xFFFF & ~data_value) + 1;
    data_value *= -1;
  }

  return data_value;
}
