#include <FastLED.h>
#include <esp_now.h>
#include <WiFi.h>

// Rotary Encoder States
#define NO_CHANGE 0
#define TURN_CW 1
#define TURN_CCW 2

// LED Effects
#define FRAMES_PER_SECOND 120

// pins
const byte DATA_PIN = 11; // neo-pixel data pin
const unsigned int ROTARY_ENC_PIN_A = 33;
const unsigned int ROTARY_ENC_PIN_B = 34;
const unsigned int ROTARY_ENC_SWITCH = 21;

// Direction     ┌─  ccw  ─┐  N  ┌─  cw  ─┐
// position       0  1  2  3  4  5  6  7  8
byte aState[] = {3, 2, 0, 1, 3, 2, 0, 1, 3};
byte lastState = 3;
volatile int count = 0;
unsigned int position = 4;
volatile byte encoderStatus = NO_CHANGE;

// LED array
const byte NUM_LEDS = 12;
CRGB leds[NUM_LEDS];

const int INCREMENT = 255 / NUM_LEDS; // Used to split Hue, Sat, and Val into selectable increments

typedef struct neopixel_data
{
  int hue;
  int saturation;
  int value;
} neopixel_data;

// Where Neopixel data is stored
neopixel_data data = {120, 120, 255};

CRGBPalette16 pacifica_palette_1 =
    {0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117,
     0x000019, 0x00001C, 0x000026, 0x000031, 0x00003B, 0x000046, 0x14554B, 0x28AA50};
CRGBPalette16 pacifica_palette_2 =
    {0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117,
     0x000019, 0x00001C, 0x000026, 0x000031, 0x00003B, 0x000046, 0x0C5F52, 0x19BE5F};
CRGBPalette16 pacifica_palette_3 =
    {0x000208, 0x00030E, 0x000514, 0x00061A, 0x000820, 0x000927, 0x000B2D, 0x000C33,
     0x000E39, 0x001040, 0x001450, 0x001860, 0x001C70, 0x002080, 0x1040BF, 0x2060FF};

unsigned long previousMillis = 0;
unsigned int effectInterval = 10;

// Global copy of receiver
esp_now_peer_info_t receiver;

#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// Flag set in ISR to indicate a button press
volatile boolean buttonPressed = false;

/************************************************************
   ISR: Action to take on Rotary Endcode switch press
 ***********************************************************/
void buttonPress()
{
  buttonPressed = true; // flag that button was pressed
}

/************************************************************
  Debounce Rot Switch
 ***********************************************************/
void debounceRotSwitch()
{
  delay(100);
  while (!digitalRead(ROTARY_ENC_SWITCH))
    ;
  delay(100);
  buttonPressed = false;
}

/************************************************************
   ISR: Get rotary encoder position
 ***********************************************************/
void ICACHE_RAM_ATTR readEncoderStatus()
{
  byte A_Output = digitalRead(ROTARY_ENC_PIN_A);
  byte B_Output = digitalRead(ROTARY_ENC_PIN_B);
  byte currState = (A_Output * 2) + B_Output;

  if (currState != lastState)
  {

    if (currState == aState[position + 1])
    {
      position++;
      // Serial.println(position);
      if (position == 8)
      {
        count++;
        position = 4;
        encoderStatus = TURN_CW;
      }
    }
    else if (currState == aState[position - 1])
    {
      position--;
      // Serial.println(position);
      if (position == 0)
      {
        count--;
        position = 4;
        encoderStatus = TURN_CCW;
      }
    }

    lastState = currState;
  }
}

/*******************************************************
  Pacifica effect functions for standbye mode
*******************************************************/
void pacifica_loop()
{
  // Increment the four "color index start" counters, one for each wave layer.
  // Each is incremented at a different speed, and the speeds vary over time.
  static uint16_t sCIStart1, sCIStart2, sCIStart3, sCIStart4;
  static uint32_t sLastms = 0;
  uint32_t ms = GET_MILLIS();
  uint32_t deltams = ms - sLastms;
  sLastms = ms;
  uint16_t speedfactor1 = beatsin16(3, 179, 269);
  uint16_t speedfactor2 = beatsin16(4, 179, 269);
  uint32_t deltams1 = (deltams * speedfactor1) / 256;
  uint32_t deltams2 = (deltams * speedfactor2) / 256;
  uint32_t deltams21 = (deltams1 + deltams2) / 2;
  sCIStart1 += (deltams1 * beatsin88(1011, 10, 13));
  sCIStart2 -= (deltams21 * beatsin88(777, 8, 11));
  sCIStart3 -= (deltams1 * beatsin88(501, 5, 7));
  sCIStart4 -= (deltams2 * beatsin88(257, 4, 6));

  // Clear out the LED array to a dim background blue-green
  fill_solid(leds, NUM_LEDS, CRGB(2, 6, 10));

  // Render each of four layers, with different scales and speeds, that vary over time
  pacifica_one_layer(pacifica_palette_1, sCIStart1, beatsin16(3, 11 * 256, 14 * 256), beatsin8(10, 70, 130), 0 - beat16(301));
  pacifica_one_layer(pacifica_palette_2, sCIStart2, beatsin16(4, 6 * 256, 9 * 256), beatsin8(17, 40, 80), beat16(401));
  pacifica_one_layer(pacifica_palette_3, sCIStart3, 6 * 256, beatsin8(9, 10, 38), 0 - beat16(503));
  pacifica_one_layer(pacifica_palette_3, sCIStart4, 5 * 256, beatsin8(8, 10, 28), beat16(601));

  // Add brighter 'whitecaps' where the waves lines up more
  pacifica_add_whitecaps();

  // Deepen the blues and greens a bit
  pacifica_deepen_colors();
}

// Add one layer of waves into the led array
void pacifica_one_layer(CRGBPalette16 &p, uint16_t cistart, uint16_t wavescale, uint8_t bri, uint16_t ioff)
{
  uint16_t ci = cistart;
  uint16_t waveangle = ioff;
  uint16_t wavescale_half = (wavescale / 2) + 20;
  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    waveangle += 250;
    uint16_t s16 = sin16(waveangle) + 32768;
    uint16_t cs = scale16(s16, wavescale_half) + wavescale_half;
    ci += cs;
    uint16_t sindex16 = sin16(ci) + 32768;
    uint8_t sindex8 = scale16(sindex16, 240);
    CRGB c = ColorFromPalette(p, sindex8, bri, LINEARBLEND);
    leds[i] += c;
  }
}

// Add extra 'white' to areas where the four layers of light have lined up brightly
void pacifica_add_whitecaps()
{
  uint8_t basethreshold = beatsin8(9, 55, 65);
  uint8_t wave = beat8(7);

  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    uint8_t threshold = scale8(sin8(wave), 20) + basethreshold;
    wave += 7;
    uint8_t l = leds[i].getAverageLight();
    if (l > threshold)
    {
      uint8_t overage = l - threshold;
      uint8_t overage2 = qadd8(overage, overage);
      leds[i] += CRGB(overage, overage2, qadd8(overage2, overage2));
    }
  }
}

// Deepen the blues and greens
void pacifica_deepen_colors()
{
  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i].blue = scale8(leds[i].blue, 145);
    leds[i].green = scale8(leds[i].green, 200);
    leds[i] |= CRGB(2, 5, 7);
  }
}

/*******************************
   Set Hue
 *******************************/
int setHue(CHSV inColor)
{

  int prevCount = 1;
  count = 0;

  while (!buttonPressed)
  { // Keep running until button is pressed

    if (count != prevCount)
    {

      count = count > NUM_LEDS - 1 ? 0 : count;
      count = count < 0 ? NUM_LEDS - 1 : count;

      noInterrupts();
      prevCount = count;
      interrupts();

      for (int i = 0; i < NUM_LEDS; i++)
      { // Display choices
        leds[i] = CHSV(i * INCREMENT, inColor.saturation, inColor.value);
      }

      // Make selected LED brighter
      leds[prevCount] = CHSV(prevCount * INCREMENT, inColor.saturation, 255);

      // Update RX
      data.hue = prevCount * INCREMENT;
      data.saturation = inColor.saturation;
      data.value = 255;
      sendData();

      FastLED.show();
    }
  }
  debounceRotSwitch();

  return prevCount * INCREMENT;
}

/*******************************
   Set Saturation
 *******************************/
int setSaturation(CHSV inColor)
{

  int prevCount = 1;
  count = 0;

  while (!buttonPressed)
  { // Keep running until button is pressed

    // Get Count
    if (count != prevCount)
    {

      count = count > NUM_LEDS - 1 ? 0 : count;
      count = count < 0 ? NUM_LEDS - 1 : count;

      noInterrupts();
      prevCount = count;
      interrupts();

      // Display choices
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i].setHSV(inColor.hue, i * INCREMENT, inColor.value);
      }

      // Make selected LED brighter
      leds[prevCount].setHSV(inColor.hue, prevCount * INCREMENT, 255);

      // Update RX
      data.hue = inColor.hue;
      data.saturation = prevCount * INCREMENT;
      data.value = 255;
      sendData();

      FastLED.show();
    }
  }

  debounceRotSwitch();

  return prevCount * INCREMENT;
}

/*******************************
   Set Value
 *******************************/
int setValue(CHSV inColor)
{

  int prevCount = 1;
  count = 5;

  int tempVal = 5;

  while (!buttonPressed)
  { // Keep running until button is pressed

    // Get Count
    if (count != prevCount)
    {

      count = count > NUM_LEDS - 1 ? NUM_LEDS - 1 : count;
      count = count < 0 ? 0 : count;

      noInterrupts();
      prevCount = count;
      interrupts();

      // Display current brightness
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CHSV(inColor.hue, inColor.saturation, prevCount * INCREMENT);
      }

      // Update RX
      data.hue = inColor.hue;
      data.saturation = inColor.saturation;
      data.value = prevCount * INCREMENT;
      sendData();

      Serial.print("Val -> ");
      Serial.println(prevCount * INCREMENT);

      FastLED.show();
    }
  }
  debounceRotSwitch();
  return prevCount * INCREMENT;
}

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
  leds[pos] += CHSV(255, 255, 255);
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

bool manageReceiver()
{
  if (receiver.channel == CHANNEL)
  {
    if (DELETEBEFOREPAIR)
    {
      deletePeer();
    }

    Serial.print("Receiver Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(receiver.peer_addr);
    if (exists)
    {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    }
    else
    {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&receiver);
      if (addStatus == ESP_OK)
      {
        // Pair success
        Serial.println("Pair success");
        return true;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT)
      {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_ARG)
      {
        Serial.println("Invalid Argument");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_FULL)
      {
        Serial.println("Peer list full");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NO_MEM)
      {
        Serial.println("Out of memory");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_EXIST)
      {
        Serial.println("Peer Exists");
        return true;
      }
      else
      {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  }
  else
  {
    // No slave found to process
    Serial.println("No Receiver found to process");
    return false;
  }
}

void deletePeer()
{
  esp_err_t delStatus = esp_now_del_peer(receiver.peer_addr);
  Serial.print("Receiver Delete Status: ");
  if (delStatus == ESP_OK)
  {
    // Delete success
    Serial.println("Success");
  }
  else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  }
  else if (delStatus == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Not sure what happened");
  }
}

void scanMode()
{
  while (!ScanForRXs()) // Wait until RX is found
  {
    for (int i = 0; i < 255; i++)
    {
      randomReds();
      FastLED.delay(1000 / FRAMES_PER_SECOND);
      FastLED.show();
    }

    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
  };

  while (!manageReceiver()) // Wait until RX is found
  {
    for (int i = 0; i < 255; i++)
    {
      randomReds();
      FastLED.delay(1000 / FRAMES_PER_SECOND);
      FastLED.show();
    }

    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
  };
}

// send data
void sendData()
{
  const uint8_t *peer_addr = receiver.peer_addr;
  Serial.print("Sending: "); // Serial.println(data);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&data, sizeof(data));
  Serial.print("Send Status: ");
  if (result == ESP_OK)
  {
    Serial.println("Success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Not sure what happened");
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{

  Serial.begin(115200);

  pinMode(ROTARY_ENC_SWITCH, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_SWITCH), buttonPress, RISING);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_PIN_A), readEncoderStatus, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_PIN_B), readEncoderStatus, CHANGE);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();

  WiFi.mode(WIFI_STA);
  InitESPNow();
  esp_now_register_send_cb(OnDataSent);

  scanMode();
}

void loop()
{

  unsigned long currentMillis = millis();

  bool standbye = true;
  bool selection = false;
  bool scan = false;

  // Standbye Mode -> Pacifica Effect
  if (standbye && currentMillis - previousMillis > effectInterval)
  {
    pacifica_loop();
    FastLED.show();

    if (buttonPressed)
    {
      debounceRotSwitch();

      standbye = false;
      selection = true;
    }
  }

  // Selection Mode
  if (selection)
  {
    CHSV startColor = CHSV(255, 255, 150);

    int hue_t = setHue(startColor);
    startColor.hue = hue_t;

    int sat_t = setSaturation(startColor);
    startColor.saturation = sat_t;

    int val = setValue(startColor);
    startColor.value = val;

    standbye = true;
    selection = false;
  }
}