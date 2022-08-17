#include <FastLED.h>
#include <esp_now.h>
#include <WiFi.h>

// Global copy of slave
esp_now_peer_info_t slave;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// pins
const byte DATA_PIN = 11; // neo-pixel data pin
const unsigned int ROTARY_ENC_PIN_A = 33;
const unsigned int ROTARY_ENC_PIN_B = 34;
const unsigned int ROTARY_ENC_SWITCH = 21;

// Rotary Encoder States
#define NO_CHANGE 0
#define TURN_CW 1
#define TURN_CCW 2

// Direction     ┌─  ccw  ─┐  N  ┌─  cw  ─┐
// position       0  1  2  3  4  5  6  7  8
byte aState[] = {3, 2, 0, 1, 3, 2, 0, 1, 3};
byte lastState = 3;
volatile int count = 0;
unsigned int position = 4;
volatile byte encoderStatus = NO_CHANGE;

// LED array
const byte NUM_LEDS = 12;
const int INCREMENT = 255 / NUM_LEDS; // Used to split Hue, Sat, and Val into selectable increments
CRGB leds[NUM_LEDS];

typedef struct neopixel_data
{
  int hue;
  int saturation;
  int value;
} neopixel_data;

// Where Neopixel data is stored
neopixel_data data = {120, 120, 255};

CHSV colorOut = CHSV(255, 255, 150);

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

/*******************************
   Set Hue
 *******************************/
int setHue(CHSV inColor)
{

  int prevCount = 1;
  count = 0;

  debounceRotSwitch();

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
  buttonPressed = false; // Reset button press

  return prevCount * INCREMENT;
}

/*******************************
   Set Saturation
 *******************************/
int setSaturation(CHSV inColor)
{

  int prevCount = 1;
  count = 0;

  debounceRotSwitch();

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

  buttonPressed = false; // Reset button press

  return prevCount * INCREMENT;
}

/*******************************
   Set Value
 *******************************/
int setValue(CHSV inColor)
{

  debounceRotSwitch();

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
  buttonPressed = false; // Reset button press
  return prevCount * INCREMENT;
}

// Init ESP Now with fallback
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

// Scan for slaves in AP mode
void ScanForSlave()
{
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

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
        Serial.println("Found a Slave.");
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
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
        {
          for (int ii = 0; ii < 6; ++ii)
          {
            slave.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0;       // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound)
  {
    Serial.println("Slave Found, processing..");
  }
  else
  {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave()
{
  if (slave.channel == CHANNEL)
  {
    if (DELETEBEFOREPAIR)
    {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if (exists)
    {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    }
    else
    {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
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
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer()
{
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
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

// send data
void sendData()
{
  const uint8_t *peer_addr = slave.peer_addr;
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

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV(data.hue, data.saturation, data.value);
  }
  FastLED.show();

  pinMode(ROTARY_ENC_SWITCH, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_SWITCH), buttonPress, RISING);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_PIN_A), readEncoderStatus, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_PIN_B), readEncoderStatus, CHANGE);

  // Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
}

void loop()
{
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL)
  { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired && buttonPressed)
    {
      // pair success or already paired
      // Send data to device
      int hue = setHue(colorOut);
      colorOut.hue = hue;

      int sat = setSaturation(colorOut);
      colorOut.saturation = sat;

      int val = setValue(colorOut);
      colorOut.value = val;

      buttonPressed = false;
    }
    else
    {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }
  else
  {
    // No slave found to process
  }

  // wait for 3seconds to run the logic again
  delay(3000);
}