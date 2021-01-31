/*
 Name:    Seven_Segment_Clock.ino
 Created: 1/8/2021 9:08:17 PM
 Author:  kerip
*/

// the setup function runs once when you press reset or power the board

#include <Adafruit_NeoPixel.h>
#include <M5Stack.h>

// ~~~ LEDs ~~~
#define PIN_LED_STRIP_S 27
//led count (14 +14 + 2 + 14 + 14)
#define LED_STRIP_N 58
#define LED_PER_SEG 14
#define LED_PER_SEP 14

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_STRIP_N, PIN_LED_STRIP_S, NEO_GRB + NEO_KHZ400);
//https://core-electronics.com.au/tutorials/ws2812-addressable-leds-arduino-quickstart-guide.html
//Set the colours -> strip.Color(r, g, b);
uint32_t colour_off = strip.Color(0, 0, 0);
uint32_t colour_purple = strip.Color(127, 0, 255); //purple
uint32_t colour_green = strip.Color(0, 255, 0); //green
uint32_t colour_red = strip.Color(255, 0, 0); //red
uint32_t colour_blue = strip.Color(0, 0, 255); //blue
uint32_t colour_warmwhite = strip.Color(239, 235, 216); //warm white
uint32_t colour_selected = strip.Color(0, 0, 0);

uint32_t COL_LED_SEG_DEFAULT = colour_warmwhite;
uint32_t COL_LED_SEG_WORK = colour_red;
uint32_t COL_LED_SEG_REST = colour_blue;
uint32_t COL_LED_SEG_EDIT = colour_purple;
uint32_t currentColour = COL_LED_SEG_DEFAULT;

// ~~~ LCD SCREEN ~~~
//lcd pixels (320 x 240)
const int LCD_PX_X = 320;
const int LCD_PX_Y = 240;
const int LCD_PX_X_MID = LCD_PX_X / 2;
const int LCD_PX_Y_MID = LCD_PX_Y / 2;

//COlurs
const int COL_BACKGROUND = TFT_BLACK;
const int COL_SEGMENTS = TFT_BLACK;


//Number segments - LED trip
//const int LED_SEG_ZERO[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
int LED_SEG_ZERO[] = { 1,1,1,1,1,1,1,1,1,1,1,1,0,0 };
long LED_SEG_ZERO_B = 0b00111111111111;
//const int LED_SEG_ONE[] = { 1, 2, 11, 12 };
int LED_SEG_ONE[] = { 1,1,0,0,0,0,0,0,0,0,1,1,0,0};
long LED_SEG_ONE_B = 0b00110000000011;
//const int LED_SEG_TWO[] = { 3, 4, 5, 6, 9, 10, 11, 12, 13, 14 };
int LED_SEG_TWO[] = { 0,0,1,1,1,1,0,0,1,1,1,1,1,1 };
long LED_SEG_TWO_B = 0b11111100111100;
int LED_SEG_THREE[] = { 1,1,1,1,0,0,0,0,1,1,1,1,1,1 };
long LED_SEG_THREE_B = 0b11111100001111;
int LED_SEG_FOUR[] = { 1,1,0,0,0,0,1,1,0,0,1,1,1,1 };
long LED_SEG_FOUR_B = 0b11110011000011;
int LED_SEG_FIVE[] = { 1,1,1,1,0,0,1,1,1,1,0,0,1,1 };
long LED_SEG_FIVE_B = 0b11001111001111;
int LED_SEG_SIX[] = { 1,1,1,1,1,1,1,1,1,1,0,0,1,1 };
long LED_SEG_SIX_B = 0b11001111111111;
int LED_SEG_SEVEN[] = { 1,1,0,0,0,0,0,0,1,1,1,1,0,0 };
long LED_SEG_SEVEN_B = 0b00111100000011;
int LED_SEG_EIGHT[] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
long LED_SEG_EIGHT_B = 0b11111111111111;
int LED_SEG_NINE[] = { 1,1,0,0,0,0,1,1,1,1,1,1,1,1 };
long LED_SEG_NINE_B = 0b11111111000011;

long LED_SEG_U = 0b00110011111111;
long LED_SEG_P = 0b11111111110000;
long LED_SEG_D = 0b11110000111111;
long LED_SEG_O = 0b00111111111111;
long LED_SEG_W = 0b00110011001100;
long LED_SEG_N = 0b00111111110011;

// initial Time display is 12:59:45 PM
int h = 12;
int m = 59;
int s = 45;
int s1 = 0;
int s2 = 0;
int m1 = 0;
int m2 = 0;
int flag = 1; //PM
struct timeComp
{
  int hours = 0;
  int mins = 0;
  int secs = 0;
};


// For accurate Time reading, use Arduino Real Time Clock and not just delay()
static uint32_t last_time, nowSeconds = 0; // RTC
static uint32_t last_time_seconds = 0;
static char time_str[12];
static uint32_t startMillis = 0;
static uint32_t previousMillis = 0;

//Modes
struct timer
{
  bool down_up = 0;
  int sets = 0;
  uint32_t time_work = 0;
  uint32_t time_rest = 0;
};
timer mTimer;
uint32_t time_target = 0;

enum Sequence
{
  SETUP_BEGIN,
  SETUP_DIRECTION,
  SETUP_TIME_WORK_S,
  SETUP_TIME_WORK_M,
  SETUP_TIME_REST_S,
  SETUP_TIME_REST_M,
  WAIT,
  WORK,
  REST,
  CHECKSTOP
};
Sequence seq = WAIT;
bool busyRunningTimer = false;
bool timeIsFinished = false;

//BUtton
const int PIN_BUTTON_OK = 19;
const int PIN_BUTTON_DOWN = 18;
const int PIN_BUTTON_UP = 23; // up button (or +)
//int buttonState;             // the current reading from the input pin
//int lastButtonState = 0;   // the previous reading from the input pin
//unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
struct mButton
{
  bool buttonState = 0;
  int lastButtonState = 0;
  uint32_t lastDebounceTime = 0;
};
mButton button_ok;
mButton button_up;
mButton button_down;

void setup() {
  //M5.begin(true, false, true);
  //M5.Power.begin();

  Serial.begin(115200);

  //Setup 1 second timer interrupt
  /*timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);*/

  //LED Strip setup
  strip.begin();
  strip.setBrightness(20);
  strip.show(); // Initialize all pixels to 'off'
  LEDsOff();

  //Deactivate the speaker (M5 stack)
  dacWrite(25, 0);

  //lcd (M5 stack)
  //M5.Lcd.fillScreen(COL_BACKGROUND);
  
  //RTC
  nowSeconds = millis()/1000; // read RTC initial value  

  //setup timer for testing...
  mTimer.down_up = true;
  mTimer.sets = 1;
  mTimer.time_work = 30;
  mTimer.time_rest = 10;
  seq = SETUP_BEGIN;

  //button
  pinMode(PIN_BUTTON_OK, INPUT);
  pinMode(PIN_BUTTON_UP, INPUT);
  pinMode(PIN_BUTTON_DOWN, INPUT);

  Serial.println(F("SETUP COMPLETE!"));
}

// the loop function runs over and over again until power down or reset
void loop() {

  //int reading = digitalRead(PIN_BUTTON);
  //Serial.print("digital pin: ");
  //Serial.println(digitalRead(PIN_BUTTON));



  if (busyRunningTimer)
  {
    uint32_t currentMillis = millis();
    uint32_t timeElapsed = (currentMillis - startMillis)/1000;
    nowSeconds = currentMillis / 1000;

    
      //Serial.print("time_target_ms: ");
      //Serial.println((time_target*1000), DEC);
    
    // How much time has passed, accounting for rollover with subtraction!
    if (timeElapsed <= time_target) {

      // if counting down instead of up
      if (!mTimer.down_up) {
        timeElapsed = time_target - timeElapsed;
      }
      GetTimeComponents(timeElapsed);
      
      if (s != last_time_seconds)
      {
        Serial.println(time_str);
        Serial.print("timeElapsed: ");
        Serial.println(timeElapsed, DEC);
        
        // Split the seconds into separate digits
        s1 = s % 10;
        s2 = s / 10;
        m1 = m % 10;
        m2 = m / 10;

        LED_ShowNumber(s1, 0, currentColour);
        LED_ShowNumber(s2, 14, currentColour);
        LED_ShowSeparator(true, 28, currentColour);
        LED_ShowNumber(m1, 30, currentColour);
        LED_ShowNumber(m2, 44, currentColour);

      }
      last_time_seconds = s;
    }
    else {
      //Time is finished
      timeIsFinished = true;
      busyRunningTimer = false;
      Serial.println("Timer finsihed.");
    }

    //previousMillis = currentMillis;
  }
  

  //SETUP_TIME_WORK_M,
  //SETUP_TIME_REST_S,
  //SETUP_TIME_REST_M,
  switch (seq)
  {
  case SETUP_BEGIN:
    LED_ShowUpDown(mTimer.down_up);
    seq = SETUP_DIRECTION;
    break;
    
  case SETUP_DIRECTION:
    if (checkButton(PIN_BUTTON_UP, &button_up) || checkButton(PIN_BUTTON_DOWN, &button_down))
    { 
      mTimer.down_up = !mTimer.down_up; //Adjust up down setting 
      LED_ShowUpDown(mTimer.down_up); //Display up down as words
    }    
    
    if (checkButton(PIN_BUTTON_OK, &button_ok)) //Prepare and go to next state
    {
      Serial.println("Setup work time: seconds...");
      LEDsOff();
      LED_DisplayFullTime(mTimer.time_work,COL_LED_SEG_EDIT,COL_LED_SEG_WORK, COL_LED_SEG_WORK);
      seq = SETUP_TIME_WORK_S;
    }
    break;

  case SETUP_TIME_WORK_S:
    if (checkButton(PIN_BUTTON_UP, &button_up))
    { 
      mTimer.time_work++;
      LED_DisplayFullTime(mTimer.time_work,COL_LED_SEG_EDIT,COL_LED_SEG_WORK, COL_LED_SEG_WORK);
    }
    else if(checkButton(PIN_BUTTON_DOWN, &button_down)){
      mTimer.time_work--;
      LED_DisplayFullTime(mTimer.time_work,COL_LED_SEG_EDIT,COL_LED_SEG_WORK, COL_LED_SEG_WORK);
    }
    
    if (checkButton(PIN_BUTTON_OK, &button_ok))
    {
      Serial.println("Setup work time: minutes...");
      LEDsOff();
      LED_DisplayFullTime(mTimer.time_work,COL_LED_SEG_WORK,COL_LED_SEG_EDIT, COL_LED_SEG_WORK);
      seq = SETUP_TIME_WORK_M;
    }
    break;
    
  case SETUP_TIME_WORK_M:
    if (checkButton(PIN_BUTTON_UP, &button_up))
    { 
      mTimer.time_work = mTimer.time_work + 60; //Add 1 minute
      LED_DisplayFullTime(mTimer.time_work,COL_LED_SEG_WORK,COL_LED_SEG_EDIT, COL_LED_SEG_WORK);
    }
    else if(checkButton(PIN_BUTTON_DOWN, &button_down)){
      if(mTimer.time_work >= 60){
        mTimer.time_work = mTimer.time_work - 60; //Subtract 1 minute
      }      
      LED_DisplayFullTime(mTimer.time_work,COL_LED_SEG_WORK,COL_LED_SEG_EDIT, COL_LED_SEG_WORK);
    }
    
    if (checkButton(PIN_BUTTON_OK, &button_ok))
    {
      Serial.println("Setup rest time: seconds...");
      LEDsOff();
      LED_DisplayFullTime(mTimer.time_rest,COL_LED_SEG_EDIT,COL_LED_SEG_REST, COL_LED_SEG_REST);
      seq = SETUP_TIME_REST_S;
    }
    break;  

  case SETUP_TIME_REST_S:
    if (checkButton(PIN_BUTTON_UP, &button_up))
    { 
      mTimer.time_rest++;
      LED_DisplayFullTime(mTimer.time_rest,COL_LED_SEG_EDIT,COL_LED_SEG_REST, COL_LED_SEG_REST);
    }
    else if(checkButton(PIN_BUTTON_DOWN, &button_down)){
      mTimer.time_rest--;
      LED_DisplayFullTime(mTimer.time_rest,COL_LED_SEG_EDIT,COL_LED_SEG_REST, COL_LED_SEG_REST);
    }
    
    if (checkButton(PIN_BUTTON_OK, &button_ok))
    {
      Serial.println("Setup rest time: minutes...");
      LEDsOff();
      LED_DisplayFullTime(mTimer.time_rest,COL_LED_SEG_REST,COL_LED_SEG_EDIT, COL_LED_SEG_REST);
      seq = SETUP_TIME_REST_M;
    }
    break;

  case SETUP_TIME_REST_M:
    if (checkButton(PIN_BUTTON_UP, &button_up))
    { 
      mTimer.time_rest = mTimer.time_rest + 60; //Add 1 minute
      LED_DisplayFullTime(mTimer.time_rest,COL_LED_SEG_REST,COL_LED_SEG_EDIT, COL_LED_SEG_REST);
    }
    else if(checkButton(PIN_BUTTON_DOWN, &button_down)){
      if(mTimer.time_rest >= 60){
        mTimer.time_rest = mTimer.time_rest - 60; //Subtract 1 minute
      }      
      LED_DisplayFullTime(mTimer.time_rest,COL_LED_SEG_REST,COL_LED_SEG_EDIT, COL_LED_SEG_REST);
    }
    
    if (checkButton(PIN_BUTTON_OK, &button_ok))
    {
      Serial.println("Waiting to start...");
      LEDsOff();
      seq = WAIT;
    }
    break;  
    
  case WAIT:
    //wait for button press
    if (checkButton(PIN_BUTTON_OK, &button_ok))
    {
      Serial.println("Start timer!");
      time_target = mTimer.time_work;
      currentColour = COL_LED_SEG_WORK;
      seq = WORK;
      busyRunningTimer = true;
      timeIsFinished = false;
      startMillis = millis();
    }
    break;
  case WORK:
    //time_target = mTimer.time_work;
    
    if (timeIsFinished){
      Serial.println("Work is finished.");
      time_target = mTimer.time_rest;
      startMillis = millis();
      currentColour = COL_LED_SEG_REST;
      seq = REST;
      timeIsFinished = false;
      busyRunningTimer = true;  
    }
    break;
  case REST:    
    if (timeIsFinished) { 
      Serial.println("Rest is finished.");
      seq = CHECKSTOP;
      timeIsFinished = true;
      busyRunningTimer = false;
      
    }
    break;
  case CHECKSTOP:
    //todo , check round count
    seq = SETUP_BEGIN;
    LEDsOff();
    break;
  default:
    break;
  }

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//BUTTON
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*bool checkButton(int pin) {

  bool buttonPressed = false;
  int reading = digitalRead(pin);
  Serial.print("digital pin: ");
  Serial.println(reading, DEC);


  /*if (reading == 1) {
    buttonPressed = true;
    Serial.println("Button pressed...");
  }*/
   /*   
  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == 1) {
        buttonPressed = true;
        Serial.println("Button pressed...");
      }
    }
  }
  lastButtonState = reading;
  return buttonPressed;
}*/

bool checkButton(int pin, mButton *button) {

  bool buttonPressed = false;
  int reading = digitalRead(pin);
  //Serial.print("digital pin: ");
  //Serial.println(reading, DEC);
      
  // If the switch changed, due to noise or pressing:
  if (reading != button->lastButtonState) {
    // reset the debouncing timer
    button->lastDebounceTime = millis();
  }

  if ((millis() - button->lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != button->buttonState) {
      button->buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (button->buttonState == 1) {
        buttonPressed = true;
        Serial.println("Button pressed...");
      }
    }
  }
  button->lastButtonState = reading;
  return buttonPressed;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//TIMER
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//t is time in seconds = millis() / 1000;
void GetTimeComponents(unsigned long t)
{
  time_str[12];
  h = t / 3600;
  t = t % 3600;
  m = t / 60;
  s = t % 60;

  //Serial.print("seconds: " + ss);
  sprintf(time_str, "%04ld:%02d:%02d", h, m, s);
  //Serial.println(time_str);
}

timeComp GetTimeComponents_B(unsigned long tt)
{
  timeComp result;
  char tt_str[12];
  result.hours = tt / 3600;
  tt = tt % 3600;
  result.mins = tt / 60;
  result.secs = tt % 60;

  //Serial.print("seconds: " + ss);
  sprintf(tt_str, "%04ld:%02d:%02d", result.hours, result.mins, result.secs);
  //Serial.println(time_str);

  return result;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// LED Animations
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void LEDsOff()
{
  for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, 0, 0, 0);         //  Set pixel's color (in RAM)                               
  }
  strip.show();
  delay(10);
}

void LED_DisplayFullTime(unsigned long tt, uint32_t colour_s, uint32_t colour_m, uint32_t colour_d){

  timeComp fullTime;
  fullTime = GetTimeComponents_B(tt);
  // Split the seconds into separate digits
  int secs1 = fullTime.secs % 10;
  int secs2 = fullTime.secs / 10;
  int min1 = fullTime.mins % 10;
  int min2 = fullTime.mins / 10;

  LED_ShowNumber(secs1, 0, colour_s);
  LED_ShowNumber(secs2, 14, colour_s);
  LED_ShowSeparator(true, 28, colour_d);
  LED_ShowNumber(min1, 30, colour_m);
  LED_ShowNumber(min2, 44, colour_m);
}

void LED_ShowSeparator(bool on, int startLED, uint32_t colour)
{
  for (int i = startLED; i < (startLED + LED_PER_SEP); i++) { // For each pixel in strip...
    if (on)
    {
      strip.setPixelColor(i, colour);
    }
    else {
      strip.setPixelColor(i, 0, 0, 0);
    }
      
  }
  strip.show();                          //  Update strip to match
  delay(10);
}

void LED_ShowNumber(int number, int startLED, uint32_t colour)
{
  int segmentArray[LED_STRIP_N];
  long segmentBinaryPattern = 0b00000000000000;

  switch (number)
  {
  case 0:
    segmentBinaryPattern = LED_SEG_ZERO_B;
    break;
  case 1:
    segmentBinaryPattern = LED_SEG_ONE_B;
    break;
  case 2:
    segmentBinaryPattern = LED_SEG_TWO_B;
    break;
  case 3:
    segmentBinaryPattern = LED_SEG_THREE_B;
    break;
  case 4:
    segmentBinaryPattern = LED_SEG_FOUR_B;
    break;
  case 5:
    segmentBinaryPattern = LED_SEG_FIVE_B;
    break;
  case 6:
    segmentBinaryPattern = LED_SEG_SIX_B;
    break;
  case 7:
    segmentBinaryPattern = LED_SEG_SEVEN_B;
    break;
  case 8:
    segmentBinaryPattern = LED_SEG_EIGHT_B;
    break;
  case 9:
    segmentBinaryPattern = LED_SEG_NINE_B;
    break;
  default:
    segmentBinaryPattern = LED_SEG_ZERO_B;
    break;
  }


  /*for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
    if (number[i] == 1)
    {
      strip.setPixelColor(i, 255, 0, 0);
    }
    else {
      strip.setPixelColor(i, 0, 0, 0);         //  Set pixel's color (in RAM)
    }
    
    strip.show();                          //  Update strip to match                               
  }*/


  long shiftedPattern = segmentBinaryPattern;
  long bit = 0;
  for (int i = startLED; i < (startLED + LED_PER_SEG); i++) { // For each pixel in strip...
    shiftedPattern = segmentBinaryPattern >> (i - startLED);
    bit = shiftedPattern & 0b00000000000001;
    if (bit == 1)
    {
      strip.setPixelColor(i, colour);
      //strip.setPixelColor(i, 255, 0, 0);
    }
    else {
      strip.setPixelColor(i, 0, 0, 0);         //  Set pixel's color (in RAM)
      //strip.setPixelColor(i, 0, 0, 0);
    }

    strip.show();                          //  Update strip to match
    delay(10);
  }  
}


void binary2led(long binaryPattern, int startLED, uint32_t colour){

  long shiftedPattern = binaryPattern;
  long bit = 0;
  for (int i = startLED; i < (startLED + LED_PER_SEG); i++) { // For each pixel in strip...
    shiftedPattern = binaryPattern >> (i - startLED);
    bit = shiftedPattern & 0b00000000000001;
    if (bit == 1)
    {
      strip.setPixelColor(i, colour);
      //strip.setPixelColor(i, 255, 0, 0);
    }
    else {
      strip.setPixelColor(i, 0, 0, 0);         //  Set pixel's color (in RAM)
      //strip.setPixelColor(i, 0, 0, 0);
    }

    strip.show();                          //  Update strip to match
    delay(10);
  }  
}

void LED_ShowUpDown(bool down_up)
{
  LEDsOff();
  
  if (down_up){ //UP
    Serial.println("Setting: UP");
    binary2led(LED_SEG_U,((LED_PER_SEG)),colour_purple);
    binary2led(LED_SEG_P,0,colour_purple);
  }else{ //DOWN
    Serial.println("Setting: DOWN");
    binary2led(LED_SEG_D,((3*LED_PER_SEG)+2),colour_purple);
    binary2led(LED_SEG_O,((2*LED_PER_SEG)+2),colour_purple);
    binary2led(LED_SEG_W,(LED_PER_SEG),colour_purple);
    binary2led(LED_SEG_N,0,colour_purple);
  }
}
