
#include <avr/interrupt.h>

const byte GATE1_PIN = 2; // PB2, physical pin 7
const byte GATE2_PIN = 3; // PB3, physical pin 2
const byte LED_PIN   = 1; // PB1, physical pin 6

volatile uint32_t overflowCount = 0;
volatile uint32_t startTicks = 0;
volatile uint32_t elapsedTicks = 0;

volatile bool started = false;
volatile bool measurementReady = false;

// Timer1 overflows every 256 ticks.
ISR(TIMER1_OVF_vect) {
  overflowCount++;
}

// Call from an ISR, where global interrupts are already disabled.
uint32_t getTimerTicks() {
  uint32_t overflows = overflowCount;
  uint8_t count = TCNT1;

  // An overflow may have happened before its ISR could run.
  if ((TIFR & (1 << TOV1)) && count < 255) {
    overflows++;
    count = TCNT1;
  }

  return overflows * 256UL + count;
}

// PB2 and PB3 share this pin-change interrupt.
ISR(PCINT0_vect) {
  static uint8_t previousPins = 0;

  uint8_t currentPins = PINB;

  bool gate1Fell =
    (previousPins & (1 << PB2)) &&
    !(currentPins & (1 << PB2));

  bool gate2Fell =
    (previousPins & (1 << PB3)) &&
    !(currentPins & (1 << PB3));

  previousPins = currentPins;

  if (measurementReady) return;

  if (!started && gate1Fell) {
    startTicks = getTimerTicks();
    started = true;
  }
  else if (started && gate2Fell) {
    elapsedTicks = getTimerTicks() - startTicks;
    measurementReady = true;
  }
}

void setup() {
  pinMode(GATE1_PIN, INPUT);
  pinMode(GATE2_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  cli();

  // Stop Timer1 while configuring it.
  TCCR1 = 0;
  TCNT1 = 0;
  overflowCount = 0;

  // Clear any pending Timer1 overflow flag.
  TIFR = (1 << TOV1);

  // Enable Timer1 overflow interrupt.
  TIMSK |= (1 << TOIE1);

  // Initialize the previous pin state before enabling interrupts.
  PCMSK |= (1 << PCINT2) | (1 << PCINT3);
  GIFR = (1 << PCIF);
  GIMSK |= (1 << PCIE);

  // Synchronous CPU clock / 8.
  // At 8 MHz CPU clock, Timer1 runs at 1 MHz.
  TCCR1 = (1 << CS13);

  sei();
}

void loop() {
  if (measurementReady) {
    // Copy the 32-bit result atomically.
    cli();
    uint32_t result = elapsedTicks;
    sei();

    // A completed measurement turns the LED on.
    digitalWrite(LED_PIN, HIGH);

    // For now, inspect 'result' with a debugger.
    // The expected unit is timer ticks (nominally 1 us).
    (void)result;

    while (true) {
      // One-shot test. Reset the ATtiny to measure again.
    }
  }
}