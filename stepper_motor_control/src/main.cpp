

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
RPI_PICO_Timer ITimer0(0);

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

volatile int direction = 1; // 1 for CW, -1 for CCW

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
volatile int tDelay = 0; // in micros
volatile int *tDelays = NULL;

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

  if (remainingSteps > 0)
  {
    // Serial << "remainingSteps: " << remainingSteps << "\n";
    digitalWrite(DIR_PIN, direction == 1 ? HIGH : LOW);

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(tDelays[remainingSteps] / 2.0);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(tDelays[remainingSteps] / 2.0);

    remainingSteps--;

    if (remainingSteps > 0)
    {
      ITimer0.setInterval(tDelays[remainingSteps], TimerHandler0);
    }
  }
  else
  {
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
    // Stop the interrupt
    // ITimer0.detachInterrupt();
  }

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

  while (!Serial)
    ;

  delay(100);

  Serial.print(F("\nStarting Change_Interval on "));
  Serial.println(BOARD_NAME);
  Serial.println(RPI_PICO_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));

  tDelay = (int)round(ss / vmax * 1e6); // in micros

  Serial << "tDelay: " << tDelay << '\n'; // Debug print statement

  if (ITimer0.attachInterruptInterval(50, TimerHandler0))
  {
    // Serial << "timer callback attached "
    //        << "\n";
    // Serial << "Starting ITimer OK, millis() = " << millis() << '\n';
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
    // Calculate the number of steps the motor needs to move
    deltaS = targetPos - currentPos;
    absSteps = round(abs(deltaS) / ss);

    // Set direction based on the sign of the distance
    direction = (deltaS > 0) ? 1 : -1;

    // Store the number of steps in remainingSteps
    remainingSteps = absSteps;

    // Allocate memory for the tDelays array
    if (tDelays != NULL)
    {
      delete[] tDelays;
    }
    tDelays = new int[absSteps];

    float vmax_c = vmax;
    float s = 0; // current position
    float s_1 = vmax_c * vmax_c / (2 * acc);
    float s_2 = deltaS - s_1;

    if (s_1 > s_2) // if we don't even reach full speed
    {
      s_1 = deltaS / 2;
      s_2 = deltaS / 2;
      vmax_c = sqrt(deltaS * acc);
    }

    // Calculate tDelay for each step
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

      // Convert velocity to delay
      tDelays[steps] = round(ss / vcurr * 1e6); // in micros
    }

    Serial.println("Starting motor");
    // Start the timer with the tDelay for the first step
    ITimer0.attachInterruptInterval(tDelays[0], TimerHandler0);
    // ITimer0.setInterval(tDelays[0], TimerHandler0);
  }

  EVERY_N_MILLISECONDS(1000)
  {
    Serial << "main loop is running "
           << "\n";
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
