// copyright cdombroski

// print debugging info
#define SERIAL_ENABLE

// enable addressable LED support
#define LED_ENABLE

#include <PinChangeInterrupt.h>
#ifdef LED_ENABLE
#include <FastLED.h>
#endif // LED_ENABLE

// for attiny85
#if defined (__AVR_ATtiny85__)
  // ATTiny doesn't have serial capability
  #ifdef SERIAL_ENABLE
  #undef SERIAL_ENABLE
  #endif SERIAL_ENABLE

  // inputs
  #define PIN_FORWARD 0
  #define PIN_BACKWARD 1
  // ensure a 50% voltage divider is on the reset pin between the 5v and receiver output
  #define PIN_ANALOG_BRAKE A0
  #define PIN_STEERING 2
  
  // outputs
  #define PIN_ESC 4
  #define PIN_LED 3
#else
  // inputs
  #define PIN_FORWARD 4
  #define PIN_BACKWARD 6
  #define PIN_BRAKE 8
//  #define PIN_ANALOG_BRAKE A0
  #define PIN_STEERING 10
  
  // outputs
  #define PIN_ESC 3
  #define PIN_LED 2
#endif

// used when mapping 3.3v logic to 5v
#define MAX_LOGIC 681
#define HALF_LOGIC MAX_LOGIC >> 1
// max analog int value
#define ANALOG_RESOLUTION 1024

#define ESC_MIN_PULSE 1000
#define ESC_MAX_PULSE 2000
#define ESC_RESET_MICROS 500

#define SERVO_MIN_PULSE 1092
#define SERVO_MAX_PULSE 1848

#define RECEIVER_MIN_PULSE 60
#define RECEIVER_MAX_FORWARD_PULSE 2872
#define RECEIVER_MAX_BACKWARD_PULSE 1924

#define STEERING_DEADZONE (ANALOG_RESOLUTION >> 3)

#define LED_COLOR_OFF CRGB::Black
#define LED_COLOR_HEADLIGHT CRGB::White
// amber
#define LED_COLOR_BLINKER 0xFF9f00
// low brightness red
#define LED_COLOR_TAILLIGHT 0x4f0000
#define LED_COLOR_BRAKELIGHT CRGB::Red
#define LED_COLOR_REVERSELIGHT 0x7f7f7f

// ordered clockwise
#define LED_LEFT_HEADLIGHT 1
#define LED_RIGHT_HEADLIGHT 2
#define LED_LEFT_BLINKER 0
#define LED_RIGHT_BLINKER 3
#define LED_LEFT_TAILLIGHT 8
#define LED_RIGHT_TAILLIGHT 4
#define LED_LEFT_REVERSELIGHT 7
#define LED_RIGHT_REVERSELIGHT 5
#define LED_CENTER_BRAKELIGHT 6
#define NUM_LEDS 9

#define BAD_TIMING 0x7fffffff

int interruptsWereDisabled = false;
volatile long forwardRiseMicros = 0;
volatile unsigned long forwardElapsedMicros = 0;
volatile long backwardRiseMicros = 0;
volatile unsigned long backwardElapsedMicros = 0;
volatile unsigned long brakingStartMicros = 0;
volatile unsigned long steeringRiseMicros = 0;
volatile unsigned long steeringElapsedMicros = 0;

int forward = 0;
int backward = 0;
int brake = 0;
int steering = 0;
int throttle = 0;

#ifdef LED_ENABLE
CRGB leds[NUM_LEDS];
#endif // LED_ENABLE

void setup() {
  pinMode(PIN_FORWARD, INPUT);
  pinMode(PIN_BACKWARD, INPUT);
  #ifdef PIN_ANALOG_BRAKE
  pinMode(PIN_ANALOG_BRAKE, INPUT_PULLUP);
  #else // PIN_ANALOG_BRAKE
  pinMode(PIN_BRAKE, INPUT);
  #endif // PIN_ANALOG_BRAKE
  pinMode(PIN_STEERING, INPUT);
  pinMode(PIN_ESC, OUTPUT);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_FORWARD), forwardChange, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BACKWARD), backwardChange, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_STEERING), steeringChange, CHANGE);

  #ifdef LED_ENABLE
  FastLED.addLeds<NEOPIXEL, PIN_LED>(leds, NUM_LEDS);
  #endif // LED_ENABLE

  #ifdef SERIAL_ENABLE
  Serial.begin(115200);
  while (!Serial)
    ; // busy wait
  #endif // SERIAL_ENABLE
}

void loop() {
  readInputs();
  writeEsc(throttle);

  #ifdef LED_ENABLE
  writeLeds();
  #endif // LED_ENABLE
  
  #ifdef SERIAL_ENABLE
  writeSerial();
  #endif // SERIAL_ENABLE
}

void forwardChange() {
  if(interruptsWereDisabled) {
    forwardRiseMicros = BAD_TIMING;
    return;
  }
  unsigned long now = micros();
  if (digitalRead(PIN_FORWARD)) {
    // we are on rising edge
    forwardRiseMicros = now;
  } else {
    // we are on falling edge
    if (forwardRiseMicros == BAD_TIMING) {
      return;
    }
    forwardElapsedMicros = now - forwardRiseMicros;
    forwardRiseMicros = -forwardRiseMicros;
  }
}

void backwardChange() {
  if(interruptsWereDisabled) {
    backwardRiseMicros = BAD_TIMING;
    return;
  }
  unsigned long now = micros();
  if (digitalRead(PIN_BACKWARD)) {
    // we are on rising edge
    backwardRiseMicros = now;
  } else {
    // we are on falling edge
    if (backwardRiseMicros == BAD_TIMING) {
      return;
    }
    backwardElapsedMicros = now - backwardRiseMicros;
    backwardRiseMicros = -backwardRiseMicros;
  }
}

void steeringChange() {
  if(interruptsWereDisabled) {
    steeringRiseMicros = BAD_TIMING;
    return;
  }
  unsigned long now = micros();
  if (digitalRead(PIN_STEERING)) {
    // we are on rising edge
    steeringRiseMicros = now;
  } else {
    // we are on falling edge
    if (steeringRiseMicros == BAD_TIMING) {
      return;
    }
    steeringElapsedMicros = now - steeringRiseMicros;
  }
}

void readInputs() {
  unsigned long now = micros();

  // read forward
  if (forwardRiseMicros < 0 && now + forwardRiseMicros > RECEIVER_MAX_FORWARD_PULSE << 1) {
    forwardElapsedMicros = 0; // no signal in a while. reset to zero
  }
  forward = map(forwardElapsedMicros, RECEIVER_MIN_PULSE, RECEIVER_MAX_FORWARD_PULSE, 0, ANALOG_RESOLUTION);
  forward = clip(forward, 0, ANALOG_RESOLUTION);

  // read backward
  if (backwardRiseMicros < 0 && now + backwardRiseMicros > RECEIVER_MAX_BACKWARD_PULSE << 1) {
    backwardElapsedMicros = 0; // no signal in a while. reset to zero
  }
  backward = map(backwardElapsedMicros, RECEIVER_MIN_PULSE, RECEIVER_MAX_BACKWARD_PULSE, 0, ANALOG_RESOLUTION);
  backward = clip(backward, 0, ANALOG_RESOLUTION);

  // read brake
  #ifdef PIN_ANALOG_BRAKE
  // analog braking reading so we can read from a reset pin.
  // The reset pin resets at 2.2v, so half of  5v (2.5v) is plenty headroom. http://www.technoblogy.com/show?LSE
  // To wire this up, we must put the read pin in a voltage divider (5v <R1> PIN_ANALOG_BRAKE <R2> brake-output-from-receiver)
  // With a 50% split in the voltage divider, the lower read voltage will be 5v / 2 = 2.5v.
  // The upper voltage is ((5v - 3v) / 2) + 3v = 4v.
  // Hence the input will be [2.5, 4] = [2.5/5 * ANALOG_RESOLUTION, 4/5 * ANALOG_RESOLUTION] = [512, 819.2]
  brake = analogRead(PIN_ANALOG_BRAKE);
  int vMin = (2.5/5.0) * ANALOG_RESOLUTION;
  int vMax = (4.0/5.0) * ANALOG_RESOLUTION;
  // if less than half of voltage range, set 0, otherwise set full
  brake = brake < (((vMax - vMin) >> 1) + vMin) ? 0 : ANALOG_RESOLUTION;
  #else // PIN_ANALOG_BRAKE
  brake = digitalRead(PIN_BRAKE);
  #endif // PIN_ANALOG_BRAKE

  // slight filtering
  if (brake) {
    if (brakingStartMicros) {
      if (now - brakingStartMicros > RECEIVER_MAX_FORWARD_PULSE << 2) {
        // only apply brake until after it's been held down for a little while.
        // otherwise we prematurely turn it on because the throttle's not sent yet.
        brake = ANALOG_RESOLUTION;
      }
    } else {
      brakingStartMicros = now;
    }
  } else {
    brakingStartMicros = 0;
    brake = 0;
  }

  throttle = mixThrottle();

  // read steering
  steering = map(steeringElapsedMicros, SERVO_MIN_PULSE, SERVO_MAX_PULSE, 0, ANALOG_RESOLUTION);
  steering = clip(steering, 0, ANALOG_RESOLUTION);
}

int mixThrottle() {
  int throttle = 0;
  throttle += forward;
  throttle -= backward;
  if (isBraking()) {
    throttle -= brake;
  }

  throttle = clip(throttle, -ANALOG_RESOLUTION, ANALOG_RESOLUTION);
  throttle = map(throttle, -ANALOG_RESOLUTION, ANALOG_RESOLUTION, 0, ANALOG_RESOLUTION);
  
  return throttle;
}

void writeEsc(int throttle) {
  int pulseMicros = map(throttle, 0, ANALOG_RESOLUTION, ESC_MIN_PULSE, ESC_MAX_PULSE);
  // servo pcm
  unsigned long then = micros() + pulseMicros;
  digitalWrite(PIN_ESC, HIGH);
  while (micros() < then)
    ; // busy sleep. I think interrupts screw with delays
  digitalWrite(PIN_ESC, LOW);
  delayMicroseconds(ESC_RESET_MICROS);
}

#ifdef LED_ENABLE
void writeLeds() {
  leds[LED_LEFT_HEADLIGHT] = LED_COLOR_HEADLIGHT;
  leds[LED_RIGHT_HEADLIGHT] = LED_COLOR_HEADLIGHT;

  if (isBraking()) {
    leds[LED_LEFT_TAILLIGHT] = LED_COLOR_BRAKELIGHT;
    leds[LED_RIGHT_TAILLIGHT] = LED_COLOR_BRAKELIGHT;
    leds[LED_CENTER_BRAKELIGHT] = LED_COLOR_BRAKELIGHT;
  } else {
    leds[LED_LEFT_TAILLIGHT] = LED_COLOR_TAILLIGHT;
    leds[LED_RIGHT_TAILLIGHT] = LED_COLOR_TAILLIGHT;
    leds[LED_CENTER_BRAKELIGHT] = LED_COLOR_OFF;
  }

  if (isReversing()) {
    leds[LED_LEFT_REVERSELIGHT] = LED_COLOR_REVERSELIGHT;
    leds[LED_RIGHT_REVERSELIGHT] = LED_COLOR_REVERSELIGHT;
  } else {
    leds[LED_LEFT_REVERSELIGHT] = LED_COLOR_TAILLIGHT;
    leds[LED_RIGHT_REVERSELIGHT] = LED_COLOR_TAILLIGHT;
  }

  // we could probably make the blinkers start when detected instead of sticking to a rigid schedule.
  int shouldBlink = (millis() / 300) & 0x1;
  if (shouldBlink) {
    if (isSteeringLeft()) {
      leds[LED_LEFT_BLINKER] = LED_COLOR_BLINKER;
      leds[LED_LEFT_TAILLIGHT] = LED_COLOR_BRAKELIGHT;
    } else if (isSteeringRight()) {
      leds[LED_RIGHT_BLINKER] = LED_COLOR_BLINKER;
      leds[LED_RIGHT_TAILLIGHT] = LED_COLOR_BRAKELIGHT;
    }
  } else {
    leds[LED_LEFT_BLINKER] = LED_COLOR_HEADLIGHT;
    leds[LED_RIGHT_BLINKER] = LED_COLOR_HEADLIGHT;
  }

  interruptsWereDisabled = true;
  FastLED.show();
  interruptsWereDisabled = false;
}
#endif // LED_ENABLE

bool isBraking() {
  return !forward && brake;
}

bool isReversing() {
  return backward;
}

bool isSteeringLeft() {
  return steering > (ANALOG_RESOLUTION >> 1) + STEERING_DEADZONE;
}

bool isSteeringRight() {
  return steering < (ANALOG_RESOLUTION >> 1) - STEERING_DEADZONE;
}

int clip (int val, int min, int max) {
  val = val < min ? min : val;
  val = val > max ? max : val;
  return val;
}

#ifdef SERIAL_ENABLE
void writeSerial() {
  Serial.print("forward:");
  Serial.print(forward);
  Serial.print(", ");
  
  Serial.print("backward:");
  Serial.print(backward);
  Serial.print(", ");
  
  Serial.print("brake:");
  Serial.print(brake);
  Serial.print(", ");

  Serial.print("throttle:");
  Serial.print(throttle);
  Serial.print(", ");

  Serial.print("steering:");
  Serial.print(steering);
  Serial.print(", ");

  Serial.print("isBraking:");
  Serial.print(isBraking() << 9);
  Serial.print(", ");

  Serial.print("isReversing:");
  Serial.print(isReversing() << 9);
  Serial.print(", ");

  Serial.print("isSteeringLeft:");
  Serial.print(isSteeringLeft() << 9);
  Serial.print(", ");

  Serial.print("isSteeringRight:");
  Serial.print(isSteeringRight() << 9);
  
  Serial.println();
}
#endif // SERIAL_ENABLE
