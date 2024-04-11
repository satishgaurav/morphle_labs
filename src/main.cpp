// LCD2004 and Pi Pico!
#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

// define motor connections
#define DIR_PIN 2
#define STEP_PIN 3

// define microstepping pins
#define MS1_PIN 10
#define MS2_PIN 11
#define MS3_PIN 12

LiquidCrystal_I2C lcd(0x27, 16, 2);

const uint8_t ROWS = 4;
const uint8_t COLS = 4;

char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

uint8_t rowPins[ROWS] = {26, 22, 21, 20}; // Pins connected to R1, R2, R3, R4
uint8_t colPins[COLS] = {19, 18, 17, 16}; // Pins connected to C1, C2, C3, C4

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//========== PRINT OPERATOR =============
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}

template <>
inline Print &operator<<(Print &obj, float arg)
{
  obj.print(arg, 4);
  return obj;
}

// Motor constants
const int spr = 200;  // steps/revolution
const float pl = 8.0; // pitch length mm/revolution
int ms = 16;          // microsteps

int direction = 1; // 1 for CW, -1 for CCW

float inputPos = 100;
float currentPos = 0;
float targetPos = 100;

// parameters
float ss = 0; // step size in mm
int absSteps = 0;
float vmax = 100;   // max velocity in mm / s
float acc = 400;    // max acceleration in mm/(s*s)
float deltaS = 0.0; // distance to travel in mm

// current velocity --> mm / s
float vcurr = 0;

float prevVelocity = 0;
float prevDistance = 0;
float prevAccel = 0;
int prevDirection = 0;
int prevMs = 0;

bool motorStartFlag = false;

unsigned long startTime = 0;
unsigned long stopTime = 0;
unsigned long runTime = 0;
unsigned long tTime = 0;

// take user input
int getInput(const char *context, const char *unit)
{
  String inputString = "";

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter ");
  lcd.print(context);
  lcd.setCursor(0, 1);
  lcd.print("(in ");
  lcd.print(unit);
  lcd.print("):");

  while (true)
  {
    char key = keypad.getKey();

    if (key)
    {
      if (isdigit(key))
      {
        inputString += key;
        lcd.print(key);
      }
      else if (key == '*')
      {
        break;
      }
      else if (key == 'C')
      {
        inputString = ""; // Clear the input on 'C' key press
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Enter ");
        lcd.print(context);
        lcd.setCursor(0, 1);
        lcd.print("(in ");
        lcd.print(unit);
        lcd.print("):");
      }
    }
  }

  int inputValue = inputString.toInt();

  // be very careful with comparing name
  if (context == "Velocity")
  {
    if (inputValue <= 0)
    {
      inputValue = vmax;
    }
    else if (inputValue > 100)
    {
      inputValue = vmax;
    }
  }
  // be very careful with comparing name
  else if (context == "Acceleration")
  {
    if (inputValue <= 0)
    {
      inputValue = acc;
    }
    else if (inputValue > 400)
    {
      inputValue = acc;
    }
  }
  else if (context == "Microstep")
  {
    switch (inputValue)
    {
    case 1:
      inputValue = 1;
      break;
    case 2:
      inputValue = 2;
      break;
    case 4:
      inputValue = 4;
      break;
    case 8:
      inputValue = 8;
      break;
    case 16:
      inputValue = 16;
      break;
    default:
      inputValue = ms;
      break;
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(context);
  lcd.setCursor(0, 1);
  lcd.print("Value: ");
  lcd.print(inputValue);
  lcd.print(" ");
  lcd.print(unit);

  delay(1000);
  lcd.clear();
  return inputValue;
}

void setMicrostepping(int ms)
{
  // enable microstepping
  if (ms == 16)
  {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, HIGH);
  }
  else if (ms == 8)
  {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);
  }
  else if (ms == 4)
  {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);
  }
  else if (ms == 2)
  {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);
  }
  else if (ms == 1)
  {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);
  }
}

void setup()
{
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.begin(0, 2);
  // lcd.print("");
  // lcd.print("Hello World!");

  // Set up pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // microstepping pin
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);

  // Calculate steps needed for the motion
  ss = pl / (spr * ms);
  setMicrostepping(ms);

  // delay(10000);
  lcd.clear();
}

void loop()
{

  char key = keypad.getKey();

  if (key != NO_KEY)
  {
    switch (key)
    {
    // this is for direction
    case 'A':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set Direction:");
      prevDirection = direction;
      direction = 1;
      lcd.setCursor(2, 1);
      lcd.print("CW");
      delay(300);
      break;
    case 'B':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set Direction:");
      prevDirection = direction;
      direction = -1;
      lcd.setCursor(2, 1);
      lcd.print("CCW");
      delay(300);
      break;
    case '1':
      // this is for velocity
      prevVelocity = vmax;
      vmax = getInput("Velocity", "mm/s");
      if (vmax > 100)
      {
        vmax = 100;
      }
      break;
    case '2':
      // take distance input
      prevDistance = inputPos;
      inputPos = getInput("Distance", "mm");
      targetPos = inputPos;
      break;
    case '3':
      // take acceleration input
      prevAccel = acc;
      acc = getInput("Acceleration", "mm/s^2");
      if (acc > 400)
      {
        acc = 400;
      }
      break;
    case '4':
      // take acceleration input
      prevMs = ms;
      ms = getInput("Microstep", "steps");
      setMicrostepping(ms);
      ss = pl / (spr * ms);
      break;
    case '#':
      // Start the motor
      if (motorStartFlag == false)
      {
        motorStartFlag = true;
      }
      break;
    }
  }

  if (motorStartFlag)
  {

    // take absolute value of position
    deltaS = targetPos - currentPos;
    Serial << "deltaS: " << deltaS << '\n';

    absSteps = round(abs(deltaS) / ss);

    float vmax_c = vmax;

    // Set direction based on the sign of the distance
    direction = (deltaS > 0) ? 1 : -1;
    Serial << "absSteps: " << absSteps << " step size: " << ss << " direction: " << direction << "\n";
    deltaS = abs(deltaS);
    tTime = 0;

    float s = 0; // current position

    float s_1 = vmax_c * vmax_c / (2 * acc);
    float s_2 = deltaS - s_1;

    if (s_1 > s_2) // if we don't even reach full speed
    {
      s_1 = deltaS / 2;
      s_2 = deltaS / 2;
      vmax_c = sqrt(deltaS * acc);
    }

    startTime = micros();
    for (int steps = 0; steps < absSteps; steps++)
    {

      s = ((float)steps + 0.5) * ss;

      if (s < s_1)
      {
        vcurr = sqrt(2 * s * acc);
      }
      else if (s < s_2)
      {
        vcurr = vmax_c;
      }
      else
      {
        vcurr = sqrt(vmax_c * vmax_c - 2 * (s - s_2) * acc);
      }

      // Serial << "s: " << s << " s_1: " << s_1 << " s_2: " << s_2 << " vcurr: " << vcurr << "\n";

      // convert velocity to delay
      int tDelay = round(ss / vcurr * 1e6); // in micros
      tTime += tDelay;

      // EVERY_N_MILLISECONDS(100)
      // {
      //   Serial << "absSteps: " << absSteps << " s1: " << s_1 << "vcurr: " << vcurr << " tDelay: " << tDelay << " current pos: " << s << "\n";
      // }

      digitalWrite(DIR_PIN, direction == 1 ? HIGH : LOW);

      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(tDelay / 2.0);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(tDelay / 2.0);
    }

    stopTime = micros();

    // this is the current position
    if (direction == 1)
    {
      currentPos += deltaS;
    }
    else
    {
      currentPos -= deltaS;
    }

    // finally calculate the total elapsed time
    runTime = stopTime - startTime;
    motorStartFlag = false;
    lcd.clear();
  }

  // print the input
  if (prevDirection != direction || prevDistance != inputPos || prevVelocity != vmax || prevAccel != acc || prevMs != ms)
  {

    Serial << "Direction: " << direction << " Distance: "
           << inputPos << " Velocity: " << vmax << " Acceleration: "
           << acc << " ms: " << ms << "\n";

    prevDirection = direction;
    prevDistance = inputPos;
    prevVelocity = vmax;
    prevAccel = acc;
    prevMs = ms;
  }

  lcd.setCursor(0, 0);
  lcd.print("Position:");
  lcd.print((int)currentPos);
  lcd.print(" mm");
  lcd.setCursor(0, 1);
  lcd.print("Time:");
  lcd.print(tTime / 1000000.0);
  // lcd.print(runTime / 1000.0);
  lcd.print(" s");

  // EVERY_N_SECONDS(1)
  // {
  //   Serial << "this is working "
  //          << "\n";
  // }
}
