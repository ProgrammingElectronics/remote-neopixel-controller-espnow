#include <FastLED.h>
#include <esp_now.h>
#include <WiFi.h>

// pins
const byte DATA_PIN = 11; // neo-pixel data pin

// LED array
const byte NUM_LEDS = 12;
CRGB leds[NUM_LEDS];

// LED Effects
uint8_t gHue = 255; // rotating "base color" used by many of the patterns
#define FRAMES_PER_SECOND 120

unsigned long previousMillis = 0;
unsigned int effectInterval = 20;

// Global copy of receiver
esp_now_peer_info_t receiver;

#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

/* Init ESP Now with fallback */
void InitESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

void randomReds()
{
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = random(0, NUM_LEDS);
  leds[pos] += CHSV(gHue, 255, 255);
}

// Scan for receivers in AP mode
bool ScanForRXs()
{
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool receiverFound = 0;
  memset(&receiver, 0, sizeof(receiver));

  Serial.println("");
  if (scanResults == 0)
  {
    Serial.println("No WiFi devices in AP Mode found");
  }
  else
  {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i)
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS)
      {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0)
      {
        // SSID of interest
        Serial.println("Found a Receiver.");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
        // Get BSSID => Mac Address of the Receiver
        int mac[6];
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
        {
          for (int ii = 0; ii < 6; ++ii)
          {
            receiver.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }

        receiver.channel = CHANNEL; // pick a channel
        receiver.encrypt = 0;       // no encryption

        receiverFound = 1;
        // we are planning to have only one receiver in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (receiverFound)
  {
    Serial.println("Receiver Found, processing..");
  }
  else
  {
    Serial.println("Receiver Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();

  return receiverFound;
}

void setup()
{

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  fill_solid(leds, NUM_LEDS, CHSV(255, 255, 255)); // solid red
  FastLED.show();

  WiFi.mode(WIFI_STA);
  InitESPNow();
  fill_solid(leds, NUM_LEDS, CHSV(85, 255, 255)); // solid green -> initialized success
  FastLED.show();

  while (!ScanForRXs()) // Wait until RX is found
  {
    for (int i = 0; i < 255; i++)
    {
      randomReds();
      FastLED.delay(1000 / FRAMES_PER_SECOND);
      FastLED.show();
    }
    
    fill_solid(leds, NUM_LEDS, CHSV(255, 255, 255)); // solid red
    FastLED.show();

  };

  Serial.begin(115200);
  delay(1000);
}

void loop()
{

  static bool standbye = true;
  static bool selection = false;
  static bool scan = false;
}