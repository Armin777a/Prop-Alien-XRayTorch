#include <Arduino.h>



// ==================== Constants ====================

// CHANGE SETTINGS
#define BLINKER_DELAY_ON            1000        // The delay between switching states in milliseconds
#define BLINKER_DELAY_OFF           500         // The delay between switching states in milliseconds


// CHANGE PIN NUMBERS
#define BLINKER_PIN                 9           // The pin that the LED is connected to
#define BLINKER_SWITCH_PIN          12          // The pin that the switch is connected to

#define BOTTOM_LED_PIN              6           // The pin that the bottom LED is connected to
#define SIDE_LED_PIN                7           // The pin that the side LED is connected to

#define SWITCH_POS1_PIN             2           // The pin that the first switch is connected to
#define SWITCH_POS2_PIN             4           // The pin that the second switch is connected to

#define POTENTIOMETER_PIN           A0          // The pin that the potentiometer is connected to
#define POTMETER_VOLTAGE_PIN        3           // The pin that the potentiometer voltage is connected to




// ==================== Function declarations ====================

void BlinkerController();                       // The function that controls the blinking of the LED
void TogglePwm(uint8_t toggle);                 // The function that toggles the PWM
void InitializeTimers();                        // Timer initialization
void CalculateOverflowLimit(uint16_t ledBlink); // Calculate the overflow limit based on the LED blink delay




// ==================== Global Variables ====================

uint8_t BlinkerState = 0;               // The state of the LED (0 = off, 1 = on)
uint8_t EnableBlinker = 0;              // The state of the blinking (0 = off, 1 = on)
uint8_t BlinkerBlock = 0;               // The state of the blinking block (0 = off, 1 = on)

volatile uint16_t timer2_overflow_counter = 0;
uint16_t OVERFLOW_LIMIT = ((BLINKER_DELAY_ON * (F_CPU / 1024)) / (1000UL * 256UL)) - 1;





// ==================== Main code ====================

void setup() {

    // Initialize pins as inputs
    pinMode(BLINKER_PIN, OUTPUT);
    pinMode(BOTTOM_LED_PIN, OUTPUT);
    pinMode(SIDE_LED_PIN, OUTPUT);
    pinMode(POTMETER_VOLTAGE_PIN, OUTPUT);

    // Initialize pins as outputs
    pinMode(SWITCH_POS1_PIN, INPUT_PULLUP);
    pinMode(SWITCH_POS2_PIN, INPUT_PULLUP);
    pinMode(BLINKER_SWITCH_PIN, INPUT_PULLUP);
    pinMode(POTENTIOMETER_PIN, INPUT);

    // Enable the potentiometer voltage pin (5V)
    digitalWrite(POTMETER_VOLTAGE_PIN, HIGH);

    // Initialize the timers
    InitializeTimers();
}

void loop() {

    // Read the potentiometer value and map it to a range of 0-255
    int potValue = analogRead(POTENTIOMETER_PIN);      // Read the value from the potentiometer (0-1023)
    int mappedValue = map(potValue, 8, 1015, 0, 255);  // Map the potentiometer value to a range of 0-255
    if (mappedValue < 0) mappedValue = 0;              // Make sure the value is not negative
    if (mappedValue > 255) mappedValue = 255;          // Make sure the value is not higher than 255

    // Read the switch positions
    bool switchPos1 = digitalRead(SWITCH_POS1_PIN) == LOW; // Read the state of the first switch (active LOW)
    bool switchPos2 = digitalRead(SWITCH_POS2_PIN) == LOW; // Read the state of the second switch (active LOW)

    // Read the blinker switch
    EnableBlinker = digitalRead(BLINKER_SWITCH_PIN) == LOW; // Read the state of the blinking switch (active LOW)

    // Check if the blinking is enabled
    if (EnableBlinker && !BlinkerBlock) {           // Check if the blinking is enabled and the blinking block is not active
        BlinkerState = 1;                           //   Set the LED state to 1
        digitalWrite(BLINKER_PIN, BlinkerState);    //   Set the LED state
        TCNT2 = 0;                                  //   Reset the timer counter
        timer2_overflow_counter = 0;                //   Reset the overflow counter
        BlinkerBlock = 1;                           //   Set the blinking block to 1
    } else if (!EnableBlinker) {                    // Check if the blinking is disabled
        BlinkerBlock = 0;                           //   Set the blinking block to 0
        digitalWrite(BLINKER_PIN, LOW);             //   Set the LED state to 0
    }


    if (switchPos1) {                       // Check if the first switch position is active
        if (mappedValue > 0) {              // Check if the potentiometer value is higher than 0
            TogglePwm(1);                   //   Toggle the PWM
        } else {                            // If the potentiometer value is 0
            TogglePwm(0);                   //   Toggle the PWM
        }
        OCR0A = mappedValue;                //   Set the bottom LED brightness based on the potentiometer value
        TogglePwm(1);                       //   Toggle the PWM
        digitalWrite(SIDE_LED_PIN, LOW);    //   Turn off the side LED
    } else if (switchPos2) {                // Check if the second switch position is active
        OCR0A = 0x00;                          //   Set the bottom LED brightness to 0
        TogglePwm(0);                          //   Toggle the PWM
        digitalWrite(SIDE_LED_PIN, HIGH);   //   Turn on the side LED
    } else {                                // If none of the switch positions are active
        OCR0A = 0x00;                          //   Set the bottom LED brightness to 0
        TogglePwm(0);                          //   Toggle the PWM
        digitalWrite(SIDE_LED_PIN, LOW);    //   Turn off the side LED
    }

    delay(10);  // Delay to prevent bouncing
}




// ==================== Interrupts ====================

// Timer 2 compare interrupt
ISR(TIMER2_COMPA_vect) {

    // Reset the timer counter
    TCNT2 = 0;

    // Increment the overflow counter
    timer2_overflow_counter++;

    // Check if the counter reached the desired limit
    if (timer2_overflow_counter >= OVERFLOW_LIMIT) {
        // Toggle the PB1 LED
        BlinkerController();

        // Reset the overflow counter
        timer2_overflow_counter = 0;
    }
}



// ==================== Function definitions ====================

// The function that controls the blinking of the LED
void BlinkerController() {

    // Check if the blinking is enabled
    if (!EnableBlinker) {
        digitalWrite(BLINKER_PIN, LOW);
        return;
    }

    // Toggle the BlinkerState between 0 and 1
    BlinkerState ^= 0x01;

    // Set the LED brightness based on the BlinkerState
    digitalWrite(BLINKER_PIN, BlinkerState);

    if (BlinkerState) {
        CalculateOverflowLimit(BLINKER_DELAY_ON);
    } else {
        CalculateOverflowLimit(BLINKER_DELAY_OFF);
    }

}


// Toggle PWM
void TogglePwm(uint8_t toggle) {
    if (toggle) {
        TCCR0A |= (1 << COM0A1);    // Enable PWM
    } else {
        TCCR0A &= ~(1 << COM0A1);   // Disable PWM
    }
}

void CalculateOverflowLimit(uint16_t ledBlink) {
    // Calculate the overflow limit
    OVERFLOW_LIMIT = ((ledBlink * (F_CPU / 1024)) / (1000UL * 256UL)) - 1;
}


// Timer initialization
void InitializeTimers() {

    // Disable global interrupts
    noInterrupts();


    // Timer 2 initialization for compare interrupt
    TCCR2A = 0x00;     // Clear control registers
    TCCR2B = 0x00;     // Clear control registers
    TCNT2 = 0x00;      // Reset the counter

    TCCR2A |= (1 << WGM21);                                 // Set the timer to CTC mode
    TCCR2B |=  (1 << CS22) | (1 << CS21) | (1 << CS20);     // Set the prescaler
    OCR2A  = 255;                                           // Set the compare value
    TIMSK2 |= (1 << OCIE2A);                                // Enable the compare interrupt


    // Timer 0 initialization for PWM
    TCCR0A = 0x00;     // Clear control registers
    TCCR0B = 0x00;     // Clear control registers
    TCNT0 = 0x00;      // Reset the counter

    TCCR0A |= (1 << WGM01) | (1 << WGM00);                  // Set the timer to Fast PWM mode
    TCCR0A |= (1 << COM0A1);                                // Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode)
    TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00);      // Set the prescaler to 256
    OCR0A = 0x00;                                           // Set the compare value



    // Enable global interrupts
    interrupts();
}