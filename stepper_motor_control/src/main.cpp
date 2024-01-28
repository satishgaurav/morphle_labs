

#include <Arduino.h>

// define motor connections
#define DIR_PIN 2
#define STEP_PIN 3

#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <FastLED.h>

// rpi pico timer interrupt
#include <RPI_PICO_TimerInterrupt.h>

#define I2C_ADDR 0x27
#define LCD_COLUMNS 20
#define LCD_LINES 2

LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);

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

// define a timer
RPI_PICO_Timer ITimer(0);

// timer function
bool ITimer_ISR()
{
  return true;
}

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
const int ms = 16;    // microsteps

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

bool motorStartFlag = false;

unsigned long startTime = 0;
unsigned long stopTime = 0;
unsigned long runTime = 0;
unsigned long tTime = 0;

// testing interrupt
volatile int remainingSteps = 0;

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
    if (inputValue > 100)
    {
      inputValue = 100;
    }
  }

  // be very careful with comparing name
  if (context == "Acceleration")
  {
    if (inputValue > 400)
    {
      inputValue = 400;
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

  delay(300);
  lcd.clear();

  return inputValue;
}

bool TimerHandler0(struct repeating_timer *t)
{
  (void)t;

  // static bool toggle0 = false;

  // // #if (TIMER_INTERRUPT_DEBUG > 0)
  // Serial.print("ITimer0: millis() = ");
  // Serial.println(millis());
  // // #endif

  // // timer interrupt toggles pin LED_BUILTIN
  // digitalWrite(LED_BUILTIN, toggle0);
  // toggle0 = !toggle0;

  return true;
}

void setup()
{
  // Set up pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Init
  lcd.init();
  lcd.backlight();

  Serial.begin(115200);
  if (ITimer.attachInterruptInterval(1000, TimerHandler0))
  {
    Serial << "Starting ITimer OK, millis() = " << millis() << '\n';
  }
  else
  {
    Serial << "Can't set ITimer correctly. Select another freq. or interval" << '\n';
  }

  // Calculate steps needed for the motion
  ss = pl / (spr * ms);
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
    case '#':
      // Start the motor
      lcd.clear();
      if (motorStartFlag == false)
      {
        motorStartFlag = true;
      }
      break;
    }
  }

  if (motorStartFlag)
  {
    startTime = millis();

    // take absolute value of position
    deltaS = targetPos - currentPos;
    Serial << "deltaS: " << deltaS << '\n';

    absSteps = round(abs(deltaS) / ss);

    float vmax_c = vmax;

    // Set direction based on the sign of the distance
    direction = (deltaS > 0) ? 1 : -1;
    Serial << "absSteps: " << absSteps << " direction: " << direction << "\n";
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

    // this is the current position
    if (direction == 1)
    {
      currentPos += deltaS;
    }
    else
    {
      currentPos -= deltaS;
    }

    stopTime = millis();
    // runTime = stopTime - startTime;
    motorStartFlag = false;
  }

  // print the input
  if (prevDirection != direction || prevDistance != inputPos || prevVelocity != vmax || prevAccel != acc)
  {

    Serial << "Direction: " << direction << " Distance: " << inputPos << " Velocity: " << vmax << " Acceleration: " << acc << "\n";

    prevDirection = direction;
    prevDistance = inputPos;
    prevVelocity = vmax;
    prevAccel = acc;
  }

  lcd.setCursor(0, 0);
  lcd.print("Position:");
  lcd.print(currentPos);
  lcd.print(" mm");
  lcd.setCursor(0, 1);
  lcd.print("Time:");
  lcd.print(tTime / 1000000.0);
  lcd.print(" s");
}
