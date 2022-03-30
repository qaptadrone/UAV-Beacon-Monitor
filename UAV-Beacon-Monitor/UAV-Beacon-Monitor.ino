// ESP32-UAV-Beacon-Monitor
// eID beacon display for UAVs following the French regulations
// Original idea Decode_balise_ESP32 by dev-fred at https://github.com/dev-fred/Decode_balise_ESP32
// Based loosely on the ESP32-WiFi-Hash-Monster by G4lile0 at https://github.com/G4lile0/ESP32-WiFi-Hash-Monster
// Written by Romain Bazile
// V1.0 - 2022-03-29
//--------------------------------------------------------------------

#include <ESP32-Chimera-Core.h>        // https://github.com/tobozo/ESP32-Chimera-Core/
#if !defined USE_M5STACK_UPDATER
// comment this out to disable SD-Updater
#define USE_M5STACK_UPDATER
#endif

#ifdef USE_M5STACK_UPDATER
#ifdef ARDUINO_M5STACK_Core2
#define M5STACK_UPDATER_MENUDELAY 5000 // how long (millis) the SDUpdater lobby is visible at boot
#else
#define M5STACK_UPDATER_MENUDELAY 0 // M5Stack Classic/Fire don't need to see the menu
#endif
#define SDU_APP_NAME "ESP32 Beacon Monitor" // title for SD-Updater UI
#include <M5StackUpdater.h> // https://github.com/tobozo/M5Stack-SD-Updater/
#endif

#if defined ESP_IDF_VERSION_MAJOR && ESP_IDF_VERSION_MAJOR >= 4
// Use LWIP stack
#include "esp_wifi.h"
#else
// Use legacy stack
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string>
#include <cstddef>
esp_err_t event_handler(void* ctx, system_event_t* event) {
  return ESP_OK;
}
#endif

#include <Preferences.h>
#include "Buffer.h"
#include "Faces.h"
#include "FS.h"
#include "SD.h"
#include "Beacons.h"

#ifdef ARDUINO_M5STACK_FIRE
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include <FastLED.h>
#pragma diagnostic pop
#define M5STACK_FIRE_NEO_NUM_LEDS 10
#define M5STACK_FIRE_NEO_DATA_PIN 15
// Define the array of leds
CRGB leds[M5STACK_FIRE_NEO_NUM_LEDS];
#endif


#define USE_SD_BY_DEFAULT true

#define DRAW_DELAY 1000 // redraw graphs every second
#define BUTTON_DEBOUNCE 150 // check button every n millis

#if CONFIG_FREERTOS_UNICORE
#define RUNNING_CORE 0
#else
#define RUNNING_CORE 1
#endif


/* ===== run-time variables ===== */
Buffer sdBuffer;
Preferences preferences;

bool useSD = USE_SD_BY_DEFAULT;
static bool SDSetupDone = false;
static bool SDReady = false;
static bool UIReady = false;
static bool WelComeTaskReady = false;

#define MAX_BEACONS 50 // max simultaneous beacons saved (*36 bytes). In case of memory problems, you can reduce this number

drone_beacon_t beacons_known[MAX_BEACONS];
uint8_t beacons_count = 0;
uint8_t beacon_shown = 0;
uint32_t lastseen = 0;

uint32_t lastDrawTime = 0;
uint32_t lastButtonTime = millis();


unsigned int bright = 100;  // default
unsigned int bright_leds = 100;  // default
unsigned int led_status = 0;
unsigned int ledPacketCounter = 0;

TFT_eSprite header = TFT_eSprite(&M5.Lcd); // 1bit   color sprite for header
TFT_eSprite beacon = TFT_eSprite(&M5.Lcd); // 1bit   color sprite for beacon

// position for header sprite
int headerPosX = 0;
int headerPosY = 0;
// dimensions for header sprite
int headerWidth  = 320;
int headerHeight = 20;

// position for beacon sprite
int beaconPosX = 0;
int beaconPosY = 22;
// dimensions for beacon sprite
int beaconWidth = 320;
int beaconHeight = 240 - beaconPosY;

uint8_t beacon_test_data[200] = {0xd7, 0x20, 0x92, 0x80, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x00,
                                 0x06, 0xcd, 0xbf, 0x2f, 0xd3, 0x00, 0xb0, 0x02, 0x09, 0x3b,
                                 0x00, 0x00, 0x50, 0x21, 0xac, 0x90, 0x02, 0x00, // wifi_pkt_rx_ctrl_t
                                 0x80, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                 0xd8, 0xa4, 0x07, 0x79, 0x5b, 0x42, 0xd8, 0xa4, 0x07, 0x79,
                                 0x5b, 0x42, 0xc0, 0x6c, 0x02, 0xd5, 0xb5, 0x2b, 0x01, 0x00,
                                 0x00, 0x00, 0x64, 0x00, 0x01, 0x04, 0x00, 0x0f, 0x41, 0x4d,
                                 0x53, 0x20, 0x54, 0x52, 0x41, 0x4e, 0x53, 0x4d, 0x49, 0x54,
                                 0x54, 0x45, 0x52,
                                 0xdd, 0x64, 0x6a, 0x5c, 0x35, 0x01,             // OUI Start
                                 0x01, 0x01, 0x01,
                                 0x02, 0x1e,
                                 0x4d, 0x4d, 0x4d, 0x4c, 0x4e, 0x4f, 0x30, 0x30, 0x30, 0x30,
                                 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
                                 0x47, 0x50, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x31, 0x32,
                                 0x03, 0x14,
                                 0x52, 0x42, 0x41, 0x5a, 0x0f,
                                 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
                                 0x04, 0x04, 0x00, 0x42, 0x8c, 0xc5,
                                 0x05, 0x04, 0xff, 0xb6, 0x81, 0x52,
                                 0x06, 0x02, 0x00, 0x8c,
                                 0x07, 0x02, 0xff, 0x9b,
                                 0x08, 0x04, 0x00, 0x42, 0x8c, 0xc6,
                                 0x09, 0x04, 0x00, 0x02, 0x21, 0x0a,
                                 0x0a, 0x01, 0x00,
                                 0x0b, 0x02, 0x01, 0x61,
                                 0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c,
                                 0x03, 0x01, 0x06, 0x00, 0x00, 0x78, 0x56
                                };

void setupWiFiPromisc()
{
  Serial.println("NVS Flash init");
  nvs_flash_init();
#if defined ESP_IDF_VERSION_MAJOR && ESP_IDF_VERSION_MAJOR >= 4
  // esp-idf 4.4 uses LWIP
  Serial.println("LWIP init");
  esp_netif_init();
#else
  Serial.println("TCP adapter init");
  tcpip_adapter_init();
#endif
  //  wificfg.wifi_task_core_id = 0;
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
#if defined ESP_IDF_VERSION_MAJOR && ESP_IDF_VERSION_MAJOR >= 4
  // esp_event_loop_init is deprecated in esp-idf 4.4
  Serial.println("[1] Skipping event loop init");
#else
  Serial.println("[1] Attaching NULL event handler");
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
#endif
  Serial.println("[2] Initing WiFi with config defaults");
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  //ESP_ERROR_CHECK(esp_wifi_set_country(WIFI_COUNTRY_EU));

  Serial.println("[3] Setting wifi storage to ram");
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  Serial.println("[4] Setting wifi mode to NULL");
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));

  Serial.println("[5] Starting WiFi");
  ESP_ERROR_CHECK(esp_wifi_start());

  Serial.println("[6] Attaching promiscuous receiver callback");
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous));

  Serial.println("[7] Activating filter for management frames only");
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
  };
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));

  Serial.println("[8] Enabling promiscuous mode");
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

  Serial.println("[9] Setting WiFi Channel to 6");
  // now switch on monitor mode
  ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE));

  Serial.println("[10] WiFi setup done");
}






// ===== main program ================================================
void setup()
{
#ifdef ARDUINO_M5STACK_FIRE
  // load these before M5.begin() so they can eventually be turned off
  FastLED.addLeds<WS2812B, M5STACK_FIRE_NEO_DATA_PIN, GRB>(leds, M5STACK_FIRE_NEO_NUM_LEDS);
  FastLED.clear();
  FastLED.show();
#endif
  M5.begin(); // this will fire Serial.begin()

#ifdef USE_M5STACK_UPDATER
  // New SD Updater support, requires the latest version of https://github.com/tobozo/M5Stack-SD-Updater/
#if defined M5_SD_UPDATER_VERSION_INT
  SDUCfg.setLabelMenu("<< Menu");
  SDUCfg.setLabelSkip("Launch");
#endif
  checkSDUpdater( SD, MENU_BIN, M5STACK_UPDATER_MENUDELAY, TFCARD_CS_PIN ); // Filesystem, Launcher bin path, Wait delay
#endif
#if defined ESP_IDF_VERSION_MAJOR && ESP_IDF_VERSION_MAJOR >= 4
  // esp-idf 4.4 uses LWIP
  Serial.println("Using an ESP Stack version >= 4");
#else
  Serial.println("Using an ESP Stack version older than v4, please consider upgrading");
#endif

  // SD card ---------------------------------------------------------
  bool toggle = false;
  unsigned long lastcheck = millis();
  M5.Lcd.fillScreen(TFT_BLACK);
  SDSetupDone = true;
  while ( !M5.sd_begin() && SDSetupDone ) {
    toggle = !toggle;
    M5.Lcd.setTextColor( toggle ? TFT_BLACK : TFT_WHITE );
    M5.Lcd.setTextDatum( MC_DATUM );
    M5.Lcd.setTextSize( 2 );
    uint8_t yPos = M5.Lcd.height() / 2;
    M5.Lcd.drawString( "INSERT SD", M5.Lcd.width() / 2, yPos);
    yPos += M5.Lcd.fontHeight();
    M5.Lcd.drawString( "Starting with no SD", M5.Lcd.width() / 2, yPos);
    yPos += M5.Lcd.fontHeight();
    M5.Lcd.drawString( "in 30s", M5.Lcd.width() / 2, yPos);
    delay( toggle ? 300 : 500 );
    // after 30secs, starts without a SDCard
    if ( lastcheck + 30000 < millis() ) {
      //      #ifdef ARDUINO_M5STACK_Core2
      //        M5.sd_end();
      //        M5.Axp.SetLcdVoltage(2500);
      //        M5.Axp.PowerOff();
      //      #elif defined(ARDUINO_M5STACK_FIRE) || defined(ARDUINO_M5Stack_Core_ESP32)
      //        M5.setWakeupButton( BUTTON_B_PIN );
      //        M5.powerOFF();
      //      #endif
      SDSetupDone = false;
    }
  }

#ifdef ARDUINO_M5STACK_Core2
  // specific M5Core2 tweaks go here
#else // M5Classic / M5Fire turn buzzer off
  M5.Speaker.write(0);
#endif

  xTaskCreatePinnedToCore( bootAnimationTask, "bootAnimationTask", 8192, NULL, 16, NULL, RUNNING_CORE);
  xTaskCreatePinnedToCore( initSpritesTask,   "initSpritesTask",   8192, NULL, 16, NULL, RUNNING_CORE);
  xTaskCreatePinnedToCore( coreTask,          "coreTask",          8192, NULL, 16, NULL, RUNNING_CORE);

#ifdef ARDUINO_M5STACK_FIRE
  xTaskCreatePinnedToCore( &blinky,           "blinky",            2500, NULL, 1,  NULL, 1);
#endif

}


// ===== main program ================================================
void loop()
{
  // no need to waste a cycle for an empty loop
  vTaskSuspend(NULL);
}


static void initSpritesTask( void* param )
{
  // Create a 1bit sprite for the header
  header.setColorDepth(1);
  if (!header.createSprite( headerWidth, headerHeight ) ) {
    log_e("Can't create header sprite");
  }
  header.setFont( &fonts::FreeSans9pt7b );
  //header.setTextSize( 1.0 );
  header.setTextColor( TFT_BLACK, TFT_DARKCYAN ); // unintuitive: use black/blue mask
  header.setTextDatum(TL_DATUM);
  header.setBitmapColor( TFT_DARKCYAN, TFT_WHITE ); // mask will be converted to this color
  header.fillSprite(TFT_DARKCYAN);

  // create a 8bits sprite for units2
  beacon.setColorDepth(8);
  if (!beacon.createSprite( beaconWidth, beaconHeight ) ) {
    log_e("Can't create beacon sprite");
  }
  beacon.setTextDatum(TL_DATUM);
  beacon.fillSprite(TFT_BLACK);

  UIReady = true;
  log_w("Leaving initSprites task !");
  vTaskDelete(NULL);
}



static void bootAnimationTask( void* param )
{
  int waitmillis = 1000; // max animation duration
  float xpos = 200;      // initial monster x position
  float ypos = 158;      // initial monster y position
  float xdir = 1;        // bounce horizontal direction
  float vcursor = 0.0;   // cursor for sine
  float vstep = 0.009;   // vspeed
  float hstep = 0.35;    // hspeed
  float vamplitude = 56; // bounce height
  float voffset = 176;   // vertical offset
  int imgId = 12;        // image ID

  M5.Lcd.clear();
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextDatum( TC_DATUM );

  M5.Lcd.setFont(&fonts::Orbitron_Light_32);
  M5.Lcd.setTextSize(1);
  uint8_t yString = 10;
  M5.Lcd.drawString( "UAV Beacon", M5.Lcd.width() / 2, yString);
  yString += M5.Lcd.fontHeight();
  M5.Lcd.drawString( "Monitor", M5.Lcd.width() / 2, yString);
  yString += M5.Lcd.fontHeight();
  M5.Lcd.setFont(&fonts::Orbitron_Light_24);
  M5.Lcd.drawString( "v1.1-2022-03-29", M5.Lcd.width() / 2, yString);

  M5.Lcd.setSwapBytes(true);
  ypos = voffset - ( abs( cos(vcursor) ) * vamplitude );
  M5.Lcd.pushImage(xpos, ypos, 64, 64, quadcopter);

  // animate the quadcopter while the sprites are being inited
  while ( waitmillis-- > 0 || ( waitmillis < 0 && !UIReady) ) {
    ypos = voffset - ( abs( cos(vcursor) ) * vamplitude );
    M5.Lcd.pushImage(xpos, ypos, 64, 64, quadcopter);
    if (  (xdir == 1  && xpos + xdir >= M5.Lcd.width() - 64)
          || (xdir == -1 && xpos + xdir < 0 ) ) {
      xdir = -xdir;
      imgId = random() % 13;
    }
    xpos += xdir * hstep;
    vcursor += vstep;
    vTaskDelay(1);
  }

  M5.Lcd.fillRect( 0, 0, 320, 120, TFT_BLACK );

  M5.Lcd.drawString( "Checking SD...", M5.Lcd.width() / 2, 40);

  if ( !sdBuffer.init() ) { // allocate buffer memory
    // TODO: print error on display
    Serial.println("Error, not enough memory for buffer");
    while (1) vTaskDelay(1);
  }

  if ( setupSD() ) {
    sdBuffer.checkFS(&SD);
    sdBuffer.pruneZeroFiles(&SD); // SD cleanup: remove zero-length pcap files from previous scans

    if ( sdBuffer.open(&SD) ) {
      Serial.println("SD CHECK OPEN");
    } else {
      Serial.println("SD ERROR, Can't create file");
      useSD = false;
    }
  } else {
    // SD setup failed, card not inserted ?
    Serial.println("SD Setup failed");
    useSD = false;
  }

  M5.Lcd.drawString( "Setting up WiFi...", M5.Lcd.width() / 2, 40);

  setupWiFiPromisc();

  WelComeTaskReady = true;
  log_d("Leaving welcome task !");
  vTaskDelete( NULL );
}


// ===== functions ===================================================

bool setupSD()
{
  if ( SDSetupDone ) return true;
  M5.sd_end();
  int attempts = 20;
  do {
    SDSetupDone = M5.sd_begin(); // SD.begin( TFCARD_CS_PIN );
  } while ( --attempts > 0 && ! SDSetupDone );

  if (!SDSetupDone ) {
    Serial.println("Card Mount Failed"); return false;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    SDSetupDone = false;
    Serial.println("No SD_MMC card attached"); return false;
  }
  Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
  SDSetupDone = true;
  return true;
}

// ===== functions ===================================================

#define TYPE_MANAGEMENT       0x00
#define TYPE_CONTROL          0x01
#define TYPE_DATA             0x02
#define SUBTYPE_BEACONS        0x08


void wifi_promiscuous(void* buf, wifi_promiscuous_pkt_type_t type)
{
  wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  wifi_pkt_rx_ctrl_t ctrl = (wifi_pkt_rx_ctrl_t)pkt->rx_ctrl;

  if (type == WIFI_PKT_MISC) return;   // wrong packet type
  if (ctrl.sig_len > 293) return; // packet too long
  uint32_t packetLength = ctrl.sig_len;
  if (type == WIFI_PKT_MGMT) packetLength -= 4;
  // fix for known bug in the IDF
  // https://github.com/espressif/esp-idf/issues/886

  unsigned int frameControl = ((unsigned int)pkt->payload[1] << 8) + pkt->payload[0];

  uint8_t version      = (frameControl & 0b0000000000000011) >> 0;
  uint8_t frameType    = (frameControl & 0b0000000000001100) >> 2;
  uint8_t frameSubType = (frameControl & 0b0000000011110000) >> 4;

  if ((type == WIFI_PKT_MGMT) && (frameSubType == SUBTYPE_BEACONS) && (version == 0)) {
    // We received a beacon

    // Try to parse it
    drone_beacon_t beacon_tmp = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t ret = unpack_payload(pkt->payload, packetLength, &beacon_tmp);
    if (ret != 0) {
      // Serial.printf("Error with received frame: %u \n",ret);
      return;
    }

    bool known = false;

    // Check if we already know this beacon if there is more than one beacon saved
    if (beacons_count > 0) {
      for (uint8_t u = 0; u < beacons_count; u++) {
        if ((beacon_tmp.use_ansi && memcmp(beacons_known[u].id_ansi, beacon_tmp.id_ansi, 20) == 0) ||
            (memcmp(beacons_known[u].id_fr, beacon_tmp.id_fr, 30) == 0))  {
          known = true;
          //Update last seen timestamp
          time(&beacons_known[u].lastreceived_timestamp);
          beacon_tmp.firstreceived_timestamp = beacons_known[u].firstreceived_timestamp; // This is for the log
          beacons_known[u].lastreceived_timestamp = beacon_tmp.lastreceived_timestamp;   // Update last seen timestamp
          if (beacon_tmp.use_ansi){
            Serial.printf("[Beacon] %s Already known, first seen at %is\n", beacon_tmp.id_ansi, beacon_tmp.firstreceived_timestamp);
          }
          else {
            Serial.printf("[Beacon] %s Already known, first seen at %is\n", beacon_tmp.id_fr, beacon_tmp.firstreceived_timestamp);
          }
          break;
        }
      }
    }

    if (!known) {
      // unknown beacon, push it in the stack
      beacons_count ++;
      // FIXME We should make sure we don't go over MAX_BEACONS
      // In case we do, we should purge the oldest beacon out of the structure and replace it by the new one
      beacons_known[beacons_count - 1] = beacon_tmp;
      Serial.print("[Beacon] New beacon");
      if (beacon_tmp.use_ansi) {
        Serial.printf(" ID ANSI %s\n", beacon_tmp.id_ansi);
      }
      else {
        Serial.printf(" ID FR %s\n", beacon_tmp.id_fr);
      }

      // Show immediately
      //beacon_shown = beacons_count - 1;
    }

    // Save to SD
    if (useSD) {
      sdBuffer.addBeacon(&beacon_tmp);
    }
  }
}



// ===== UI functions ===================================================

void draw()
{
  header.fillSprite( TFT_BLUE );
  if (beacons_count == 0) {
    header.setTextDatum(TL_DATUM);
    header.drawString("NO BEACONS YET", 2, 2);
  }
  else {
    header.setTextDatum(TL_DATUM);
    header.drawString("SHOWN: " + String(beacon_shown + 1), 2, 2);
    header.setTextDatum(TC_DATUM);
    header.drawString("VISIBLE: " + String(beacons_count), M5.Lcd.width() / 2, 2);
  }
  header.setTextDatum(TR_DATUM);
  header.drawString("SD: " + String(useSD ? "On" : "Off"), M5.Lcd.width() - 2, 2);
  header.pushSprite( headerPosX, headerPosY );

  uint8_t yPos = 4;
  uint8_t xPos = 2;
  beacon.fillSprite( TFT_BLACK );

  if (beacons_count == 0) {
    beacon.setTextColor( TFT_SKYBLUE,  TFT_BLACK );
    beacon.setFont(&fonts::FreeSans18pt7b); // Font2 FreeSans12pt7b
    beacon.setTextSize( 1 );
    beacon.setTextDatum(MC_DATUM);
    beacon.drawString( "NO BEACON", beacon.width() / 2, beacon.height() / 2 - beacon.fontHeight() / 2  );
    beacon.drawString( "DETECTED YET", beacon.width() / 2, beacon.height() / 2 + beacon.fontHeight() );
    beacon.setTextSize( 1 );
    beacon.setTextDatum(TL_DATUM);
  }
  else {
    char buf[20];
    beacon.setFont(&fonts::FreeSans12pt7b); // Font2 FreeSans12pt7b DejaVu18
    beacon.setTextColor( TFT_GREEN, TFT_BLACK );
    beacon.setTextDatum(TC_DATUM);
    // ID
    if (beacons_known[beacon_shown].use_ansi) {
      beacon.drawString( beacons_known[beacon_shown].id_ansi, beacon.width() / 2, yPos );
    }
    else {
      beacon.drawString( beacons_known[beacon_shown].id_fr, beacon.width() / 2, yPos );
    }
    yPos += beacon.fontHeight() + 5;
    beacon.setTextDatum(TL_DATUM);

    // Latitude and Longitude
    beacon.setTextColor( 0xF804,  TFT_BLACK );
    sprintf(buf, "LAT:  %.5f", double(beacons_known[beacon_shown].lat) * 0.00001);
    beacon.drawString(buf, xPos, yPos);
    yPos += beacon.fontHeight();
    sprintf(buf, "LON:  %.5f", double(beacons_known[beacon_shown].lon) * 0.00001);
    beacon.drawString(buf, xPos, yPos);

    yPos += beacon.fontHeight() + 5;

    // Altitude and Height
    beacon.setTextColor( TFT_GREENYELLOW, TFT_BLACK );
    if (beacons_known[beacon_shown].alt == 0) {
      // Altitude is not valid
      sprintf(buf, "ALT: N/A");
    }
    else {
      sprintf(buf, "ALT: %im", beacons_known[beacon_shown].alt);
    }
    beacon.drawString(buf, xPos, yPos);
    if (beacons_known[beacon_shown].height == 0) {
      // Height is not valid
      sprintf(buf, "HGT: N/A");
    }
    else {
      sprintf(buf, "HGT: %im", beacons_known[beacon_shown].height);
    }
    beacon.drawString(buf, beacon.width() / 2, yPos);

    yPos += beacon.fontHeight() + 5;

    // Speed and Bearing
    beacon.setTextColor( TFT_VIOLET,    TFT_BLACK );
    sprintf(buf, "SPD: %im/s", beacons_known[beacon_shown].speed);
    beacon.drawString(buf, xPos, yPos);
    sprintf(buf, "DIR: %ideg", beacons_known[beacon_shown].bearing);
    beacon.drawString(buf, beacon.width() / 2, yPos);


    // Home position
    yPos += beacon.fontHeight() + 5;
    beacon.setTextColor( TFT_SKYBLUE,  TFT_BLACK );
    beacon.drawString("HOME", xPos, yPos + beacon.fontHeight() / 2);
    sprintf(buf, "LAT: %.5f", double(beacons_known[beacon_shown].lat_start) * 0.00001);
    beacon.drawString(buf, xPos + 85, yPos);
    yPos += beacon.fontHeight();
    sprintf(buf, "LON: %.5f", double(beacons_known[beacon_shown].lon_start) * 0.00001);
    beacon.drawString(buf, xPos + 85, yPos);

    // Last seen
    beacon.setTextColor( TFT_SILVER,    TFT_BLACK );
    time_t now;
    time(&now);
    sprintf(buf, "Last seen: %is ago", (now - beacons_known[beacon_shown].lastreceived_timestamp));
    beacon.setTextDatum(BC_DATUM);
    beacon.drawString(buf, beacon.width() / 2, beacon.height() - 2);
  }
  beacon.pushSprite( beaconPosX, beaconPosY );

}

// ====== Core task ===================================================
void coreTask( void * p )
{
  while ( !UIReady ) vTaskDelay(1); // wait for sprites to init
  while ( !WelComeTaskReady ) vTaskDelay(1); // wait for animation to terminate

  uint32_t currentTime;

  M5.Lcd.clear();

  lastButtonTime = millis();
  M5.update();

  while (true) {
    bool needDraw = false;
    currentTime = millis();

    if ( currentTime - lastButtonTime > BUTTON_DEBOUNCE ) {
      M5.update();
      // buttons assignment :
      //  - BtnA short press => show previous beacon
      //  - BtnA long press  => enable/disable SD storage
      //  - BtnB short press => cycle through brightness values
      //  - BtnC short press => show next beacon
      //  - BtnC long press  => reset to first beacon in list
      if ( M5.BtnA.wasReleased() ) {
        if (beacon_shown == 0) {
          if (beacons_count != 0) {
            beacon_shown = beacons_count - 1;
          }
          else {
            beacon_shown = 0;
          }
        }
        else {
          beacon_shown --;
        }
        needDraw = true;
      } else if (M5.BtnA.wasReleasefor(1000)) {
        // toggle SD use
        if ( useSD ) { // in use, disable
          sdBuffer.close(&SD); // flush current buffer
          useSD = false;
          SDSetupDone = false;
          M5.sd_end();
        }
        else { // not in use, try to enable
          if ( setupSD() ) {
            if ( !sdBuffer.open(&SD) ) {
              Serial.println(" SD ERROR, Can't create file, disabling SD");
              useSD = false;
              SDSetupDone = false;
              M5.sd_end();
            }
          }
        }
        needDraw = true;
      }

      if ( M5.BtnB.wasReleased() ) {
        bright += 50;
        if (bright > 251) bright = 0;
        M5.Lcd.setBrightness(bright);
      } else if (M5.BtnB.wasReleasefor(1000)) {
        Serial.println("Test beacon injected!");
        wifi_promiscuous(&beacon_test_data, WIFI_PKT_MGMT);
      }

      if ( M5.BtnC.wasReleased() ) {
        // Switch beacon shown
        beacon_shown ++;
        if (beacon_shown >= beacons_count) {
          beacon_shown = 0;
        }
        needDraw = true;
      } else if (M5.BtnC.wasReleasefor(1000)) {
        beacons_count = 0;
        beacon_shown = 0;
        needDraw = true;
      }

      lastButtonTime = currentTime;
      if (needDraw) draw();
    }

    // maintain buffer and save to SD if necessary
    if (useSD) sdBuffer.save(&SD);

    // draw Display
    if ( currentTime - lastDrawTime > DRAW_DELAY ) {
      lastDrawTime = currentTime;
      draw();
    }
  }
  vTaskDelete(NULL);
}


#ifdef ARDUINO_M5STACK_FIRE
void blinky( void * p )
{
  while (1) {
    for (int pixelNumber = 0; pixelNumber < M5STACK_FIRE_NEO_NUM_LEDS ; pixelNumber++) {
      leds[pixelNumber].setRGB(  0, 0, 0);
      if (led_status == pixelNumber)
        leds[pixelNumber].setRGB(  0, 0, bright_leds);
    }
    led_status++;
    if (led_status > M5STACK_FIRE_NEO_NUM_LEDS)
      led_status = 0;
    FastLED.show();

    int led_delay = 1000;

    vTaskDelay(led_delay / portTICK_RATE_MS);
  }
}
#endif
