#include <FastLED.h>

//pins
const byte DATA_PIN = 11; // neo-pixel data pin
const unsigned int ROTARY_ENC_PIN_A = 33;
const unsigned int ROTARY_ENC_PIN_B = 34;
const unsigned int ROTARY_ENC_SWITCH = 21;

//Rotary Encoder States
#define NO_CHANGE 0
#define TURN_CW   1
#define TURN_CCW  2

//Direction     ┌─  ccw  ─┐  N  ┌─  cw  ─┐
//position       0  1  2  3  4  5  6  7  8
byte aState[] = {3, 2, 0, 1, 3, 2, 0, 1, 3};
byte lastState = 3;
volatile int count = 0;
unsigned int position = 4;
volatile byte encoderStatus = NO_CHANGE;

// LED array
const byte NUM_LEDS = 12;
CRGB leds[NUM_LEDS];

const int INCREMENT = 255 / NUM_LEDS;

//Color setting to send
CHSV colorOut = CHSV(255, 255, 150);

// Flag set in ISR to indicate a button press
volatile boolean buttonPressed = false;


/************************************************************
   ISR: Action to take on Rotary Endcode switch press
 ***********************************************************/
void buttonPress() {
  buttonPressed = true;  //flag that button was pressed
}

/************************************************************
   ISR: Get rotary encoder position
***********************************************************/
void ICACHE_RAM_ATTR readEncoderStatus() {
  byte A_Output = digitalRead(ROTARY_ENC_PIN_A);
  byte B_Output = digitalRead(ROTARY_ENC_PIN_B);
  byte currState = (A_Output * 2) + B_Output;

  if (currState != lastState) {

    if (currState == aState[position + 1]) {
      position++;
      if (position == 8) {
        count++;
        position = 4;
        encoderStatus = TURN_CW;
      }
    }
    else if (currState == aState[position - 1]) {
      position--;
      if (position == 0) {
        count--;
        position = 4;
        encoderStatus = TURN_CCW;
      }
    }
    lastState = currState;
  }
}


/*************************
Debounce Rot Switch
 ***************************/
void debounceRotSwitch() {
  delay(100);
  while (!digitalRead(ROTARY_ENC_SWITCH));
  delay(100);
  buttonPressed = false;
}



/*******************************
   Set Hue
 *******************************/
int setHue(CHSV currentColor) {

  int prevCount = 1;
  count = 0;

  debounceRotSwitch();

  while (!buttonPressed) { //Keep running until button is pressed

    if (count != prevCount) {

      count = count > NUM_LEDS - 1 ? 0 : count;
      count = count < 0 ? NUM_LEDS - 1 : count;

      noInterrupts();
      prevCount = count;
      interrupts();

      for (int i = 0; i < NUM_LEDS; i++)
      { // Display choices
        leds[i] = CHSV(i * INCREMENT, currentColor.saturation, currentColor.value);
      }

      // Make selected LED brighter
      leds[prevCount] = CHSV(prevCount * INCREMENT, currentColor.saturation, 255);
      FastLED.show();
    }
  }
  buttonPressed = false; // Reset button press
  return prevCount * INCREMENT;
}

/*******************************
   Set Saturation
 *******************************/
int setSaturation(CHSV inColor) {

  int prevCount = 1;
  count = 0;

  debounceRotSwitch();

  while (!buttonPressed) { //Keep running until button is pressed

    // Get Count
    if (count != prevCount) {

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
      FastLED.show();
    }
  }

  buttonPressed = false; // Reset button press

  return prevCount * INCREMENT;
}

/*******************************
   Set Value
 *******************************/
int setValue(CHSV currentColor) {

  debounceRotSwitch();

  int prevCount = 1;
  count = 5;

  int tempVal = 5;

  while (!buttonPressed) {    //Keep running until button is pressed

    // Get Count
    if (count != prevCount) {

      count = count > NUM_LEDS - 1 ? NUM_LEDS - 1 : count;
      count = count < 0 ? 0 : count;

      noInterrupts();
      prevCount = count;
      interrupts();

      // Display current brightness
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CHSV(currentColor.hue, currentColor.saturation, prevCount * INCREMENT);
      }

      Serial.print("Val -> ");
      Serial.println(prevCount * INCREMENT);

      FastLED.show();
    }
  }
  buttonPressed = false; // Reset button press
  return prevCount * INCREMENT;
}

void setup() {

  Serial.begin(115200);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  pinMode(ROTARY_ENC_SWITCH, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_SWITCH), buttonPress, RISING);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_PIN_A), readEncoderStatus, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENC_PIN_B), readEncoderStatus, CHANGE);

  int hue = setHue(colorOut);
  colorOut.hue = hue;
  Serial.print("Hue -> ");
  Serial.println(hue);

  int sat = setSaturation(colorOut);
  colorOut.saturation = sat;
  Serial.print("Sat-> ");
  Serial.println(sat);

  int val = setValue(colorOut);
  colorOut.value = val;
  Serial.print("Val -> ");
  Serial.println(val);

  char buffer[100];
  sprintf(buffer, "Hue %d, Sat %d, Val %d", colorOut.hue, colorOut.saturation, colorOut.value);
  Serial.println(buffer);

  fill_solid(leds, NUM_LEDS, CHSV(hue, sat, val));
  FastLED.show();

}




void loop() {



}
