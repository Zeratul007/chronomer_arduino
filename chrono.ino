#include <Tiny4kOLED.h>
#include <avr/interrupt.h>

const byte BUTTON_PIN = 1;  // PB1, physical pin 6
const byte GATE1_PIN  = 3;  // PB3, physical pin 2
const byte GATE2_PIN  = 4;  // PB4, physical pin 3

// Measure between the centers of the two beams.
const uint32_t GATE_DISTANCE_MM = 50;

enum State : byte {
  CALIBRATION,
  ARMED,
  RESULT
};

volatile State state = CALIBRATION;
volatile uint32_t overflowCount = 0;
volatile uint32_t startTicks = 0;
volatile uint32_t elapsedTicks = 0;
volatile bool started = false;
volatile bool measurementValid = false;
volatile uint8_t previousPins = 0;
uint8_t lastCalibrationPins = 0xFF;

ISR(TIMER1_OVF_vect) {
  overflowCount++;
}

// Called from an ISR, where global interrupts are disabled.
uint32_t getTimerTicks() {
  uint32_t overflows = overflowCount;
  uint8_t count = TCNT1;

  if (TIFR & (1 << TOV1)) {
    overflows++;
    count = TCNT1;
  }

  return overflows * 256UL + count;
}

ISR(PCINT0_vect) {
  uint8_t currentPins = PINB;

  // Clear = LOW, blocked = HIGH.
  bool gate1Rose =
    !(previousPins & (1 << PB3)) &&
     (currentPins & (1 << PB3));

  bool gate2Rose =
    !(previousPins & (1 << PB4)) &&
     (currentPins & (1 << PB4));

  previousPins = currentPins;

  if (state != ARMED) return;

  if (!started && gate2Rose) {
    // Gate 2 triggered before Gate 1.
    measurementValid = false;
    state = RESULT;
  }
  else if (!started && gate1Rose) {
    startTicks = getTimerTicks();
    started = true;
  }
  else if (started && gate2Rose) {
    elapsedTicks = getTimerTicks() - startTicks;
    measurementValid = true;
    state = RESULT;
  }
}

void enterCalibration() {
  cli();

  state = CALIBRATION;
  started = false;
  measurementValid = false;
  elapsedTicks = 0;
  previousPins = PINB;

  sei();
}

bool buttonPressed() {
  static byte lastReading = HIGH;
  static byte stableState = HIGH;
  static uint32_t lastChange = 0;

  byte reading = digitalRead(BUTTON_PIN);

  if (reading != lastReading) {
    lastChange = millis();
    lastReading = reading;
  }

  if (millis() - lastChange >= 25 &&
      reading != stableState) {
    stableState = reading;

    if (stableState == LOW) return true;
  }

  return false;
}

void showCalibration(bool gate1Blocked, bool gate2Blocked) {
  oled.clear();

  oled.setCursor(0, 0);
  oled.println("CALIBRATION");

  oled.setCursor(0, 2);
  oled.print("Gate 1: ");
  oled.println(gate1Blocked ? "BLOCKED" : "CLEAR  ");

  oled.setCursor(0, 4);
  oled.print("Gate 2: ");
  oled.println(gate2Blocked ? "BLOCKED" : "CLEAR  ");

  oled.setCursor(0, 6);

  if (!gate1Blocked && !gate2Blocked) {
    oled.println("Both clear: press");
  } else {
    oled.println("Adjust alignment");
  }
}

void showArmed(bool measuring) {
  oled.clear();
  oled.setCursor(0, 0);
  oled.println("CHRONOGRAPH");

  oled.setCursor(0, 2);

  if (measuring) {
    oled.println("GATE 1 DETECTED");
    oled.setCursor(0, 4);
    oled.println("Waiting for G2");
  } else {
    oled.println("READY");
    oled.setCursor(0, 4);
    oled.println("Waiting for G1");
  }

  oled.setCursor(0, 6);
  oled.println("Press: cancel");
}

void showResult(uint32_t ticks, bool valid) {
  oled.clear();
  oled.setCursor(0, 0);
  oled.println("MEASUREMENT");

  if (!valid || ticks == 0) {
    oled.setCursor(0, 2);
    oled.println("INVALID PASS");

    oled.setCursor(0, 4);
    oled.println("Check alignment");

    oled.setCursor(0, 6);
    oled.println("Press: retry");
    return;
  }

  // Assumes one timer tick = 1 microsecond.
  // Speed in tenths of m/s, rounded to nearest tenth.
  uint32_t speedTenths =
    (GATE_DISTANCE_MM * 10000UL + ticks / 2) / ticks;

  oled.setCursor(0, 2);
  oled.print("Speed: ");
  oled.print(speedTenths / 10);
  oled.print('.');
  oled.print(speedTenths % 10);
  oled.println(" m/s");

  oled.setCursor(0, 4);
  oled.print("Time: ");
  oled.print(ticks);
  oled.println(" us");

  oled.setCursor(0, 6);
  oled.println("Press: measure");
}

void setup() {
  pinMode(GATE1_PIN, INPUT);
  pinMode(GATE2_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize the OLED using the same configuration
  // as the successful standalone OLED test.
  oled.begin(
    128, 64,
    sizeof(tiny4koled_init_128x64),
    tiny4koled_init_128x64
  );

  oled.enableChargePump();
  oled.setFont(FONT6X8);
  oled.clear();
  oled.on();

  bool gate1Blocked = (digitalRead(GATE1_PIN)? true : false);
  bool gate2Blocked = (digitalRead(GATE2_PIN)? true : false);
  showCalibration(gate1Blocked, gate1Blocked);

  cli();

  // Timer1: retain the timer configuration from your
  // existing sketch. At 8 MHz, /8 gives 1 us per tick
  // if this clock selection is supported as configured.
  TCCR1 = 0;
  TCNT1 = 0;
  overflowCount = 0;
  TIFR = (1 << TOV1);
  TIMSK |= (1 << TOIE1);
  TCCR1 = (1 << CS12);

  previousPins = PINB;

  // Gate 1 = PB3 / PCINT3
  // Gate 2 = PB4 / PCINT4
  PCMSK |= (1 << PCINT3) | (1 << PCINT4);
  GIFR = (1 << PCIF);
  GIMSK |= (1 << PCIE);

  sei();
}

void loop() {
  if (buttonPressed()) {
    State current;

    cli();
    current = state;
    sei();

    if (current == CALIBRATION) {
      // Clear beams are LOW.
      if (!(PINB & (1 << PB3)) &&
          !(PINB & (1 << PB4))) {
        cli();

        started = false;
        measurementValid = false;
        elapsedTicks = 0;
        previousPins = PINB;
        state = ARMED;

        sei();

        //showArmed(false);
      }
    } else {
      // Cancel a measurement or return from the result.
      enterCalibration();
      //showCalibration();
    }
  }

  // Take a consistent snapshot of the ISR-shared values.
  State currentState;
  uint32_t ticks;
  bool valid;
  bool isStarted;

  cli();
  currentState = state;
  ticks = elapsedTicks;
  valid = measurementValid;
  isStarted = started;
  sei();

  // Update the display only when its content changes.
  static State lastScreen = RESULT;
  static bool lastStarted = false;
  static bool firstUpdate = true;
  static bool lastGate1Blocked = false;
  static bool lastGate2Blocked = false;
  
  // Redraw calibration only when either gate changes.
  uint8_t gatePins = PINB;
  bool gate1Blocked = gatePins & (1 << PB3);
  bool gate2Blocked = gatePins & (1 << PB4);
  
  bool calibrationChanged =
    firstUpdate ||
    gate1Blocked != lastGate1Blocked ||
    gate2Blocked != lastGate2Blocked;

  if (firstUpdate ||
    currentState != lastScreen ||
    (currentState == ARMED &&
     isStarted != lastStarted) ||
    (currentState == CALIBRATION &&
     calibrationChanged)) {

  if (currentState == CALIBRATION) {
    showCalibration(gate1Blocked, gate2Blocked);
    lastGate1Blocked = gate1Blocked;
    lastGate2Blocked = gate2Blocked;
  }
  else if (currentState == ARMED) {
    showArmed(isStarted);
  }
  else {
    showResult(ticks, valid);
  }

    lastScreen = currentState;
    lastStarted = isStarted;
    firstUpdate = false;
  }
}
