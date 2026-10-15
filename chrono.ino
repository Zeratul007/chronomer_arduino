
#include <avr/interrupt.h>

const byte GATE1_PIN = 2; // PB2, physical pin 7
const byte GATE2_PIN = 3; // PB3, physical pin 2
const byte LED_PIN   = 1; // PB1, physical pin 6
const byte BUTTON_PIN = 0; // PB0, physical pin 5

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

ISR(TIMER1_OVF_vect) {
  overflowCount++;
}

// Called from an ISR; global interrupts are already disabled.
uint32_t getTimerTicks() {
  uint32_t overflows = overflowCount;
  uint8_t count = TCNT1;

  // Handle a pending overflow that hasn't been serviced yet.
  if (TIFR & (1 << TOV1)) {
    overflows++;
    count = TCNT1;
  }

  return overflows * 256UL + count;
}

ISR(PCINT0_vect) {
  uint8_t currentPins = PINB;

  bool gate1Fell =
    (previousPins & (1 << PB2)) &&
    !(currentPins & (1 << PB2));

  bool gate2Fell =
    (previousPins & (1 << PB3)) &&
    !(currentPins & (1 << PB3));

  previousPins = currentPins;

  if (state != ARMED) return;

  if (!started && gate2Fell) {
    // Wrong order: Gate 2 triggered first.
    measurementValid = false;
    state = RESULT;
  }
  else if (!started && gate1Fell) {
    startTicks = getTimerTicks();
    started = true;
  }
  else if (started && gate2Fell) {
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

void setup() {
  pinMode(GATE1_PIN, INPUT);
  pinMode(GATE2_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  cli();

  // Configure Timer1: synchronous CPU clock / 8.
  TCCR1 = 0;
  TCNT1 = 0;
  overflowCount = 0;
  TIFR = (1 << TOV1);
  TIMSK |= (1 << TOIE1);
  TCCR1 = (1 << CS13);

  // Set initial gate states before enabling interrupts.
  previousPins = PINB;
  PCMSK |= (1 << PCINT2) | (1 << PCINT3);
  GIFR = (1 << PCIF);
  GIMSK |= (1 << PCIE);

  sei();
}

void loop() {
  if (buttonPressed()) {
    if (state == CALIBRATION) {
      // Only arm when both beams are clear.
      if (!(PINB & (1 << PB2)) &&
          !(PINB & (1 << PB3))) {
        cli();
        started = false;
        measurementValid = false;
        previousPins = PINB;
        state = ARMED;
        sei();
      }
    } else {
      // Abort or return from result to calibration.
      enterCalibration();
    }
  }

  State currentState = state;
  uint32_t result = 0;
  bool valid = false;

  if (currentState == RESULT) {
    cli();
    result = elapsedTicks;
    valid = measurementValid;
    sei();
  }

  // Status LED behavior:
  // CALIBRATION: on if either gate is blocked.
  // ARMED: blink.
  // RESULT: solid for valid measurement, blink fast if invalid.
  if (currentState == CALIBRATION) {
    bool blocked = (PINB & (1 << PB2)) ||
                   (PINB & (1 << PB3));
    digitalWrite(LED_PIN, blocked ? HIGH : LOW);
  }
  else if (currentState == ARMED) {
    digitalWrite(LED_PIN, (millis() / 250) % 2);
  }
  else if (valid) {
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    digitalWrite(LED_PIN, (millis() / 100) % 2);
  }

  // Inspect result in a debugger for now.
  // At 8 MHz CPU clock, each timer tick is nominally 1 us.
  (void)result;
}