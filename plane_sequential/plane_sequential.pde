#define  NUM_TLCS  10
#define  NUM_ROWS  7

#define TLC_PWM_PERIOD 8192
#define TLC_GSCLK_PERIOD  3

#include "Tlc5940Mux.h"

#define  PULSE_PIN(port, pin)  port |= (1 << (pin)); port &= ~(1 << (pin))

//Pin connected to latch pin (ST_CP) of 74HC595
const int latchPin = 7;
//Pin connected to clock pin (SH_CP) of 74HC595
const int clockPin = 8;
////Pin connected to Data in (DS) of 74HC595
const int dataPin = 6;

volatile uint8_t isShifting;
uint8_t shiftRow = 0;

//byte rowSequence[7] = {
//  B00000001,
//  B00000010,
//  B00000100,
//  B00001000,
//  B00010000,
//  B00100000,
//  B01000000
//};

byte rowSequence[7] = {
  B11111110,
  B11111101,
  B11111011,
  B11110111,
  B11101111,
  B11011111,
  B10111111
};

inline void Shifter_shiftRow(){ 
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, rowSequence[shiftRow]);
  digitalWrite(latchPin, HIGH); 
}

ISR(TIMER1_OVF_vect)
{
  if (!isShifting) {
    isShifting = 1;
    PORTB |= _BV(PB1);
    
    TlcMux_shiftRow(shiftRow);
    Shifter_shiftRow();
  
    // Keep filling the register with HIGH except for once every cycle
//    if(shiftRow == 6) {
//      PORTD &= ~_BV(PD6);
//    } else {
//      PORTD |= _BV(PD6);
//    }
//    PULSE_PIN(PORTD, PD7);
//    PULSE_PIN(PORTB, PB0);

//    if(shiftRow == 6) {
//      PORTD |= _BV(PD6);
//    } else {
//      PORTD &= ~_BV(PD6);
//    }
//    PULSE_PIN(PORTD, PD7);
//    PULSE_PIN(PORTB, PB0);
    
    // -- THE IMPORTANT BIT -- //
    BLANK_PORT |= _BV(BLANK_PIN);
    // 3 x 62.5ns = 187.5ns  (Blank needs to exceed 300ns to avoid shortening the GS cycle)
    __asm__("nop\n\t""nop\n\t""nop\n\t");
    XLAT_PORT |= _BV(XLAT_PIN);
    // XLAT for 62.5 ns
    __asm__("nop\n\t");
    XLAT_PORT &= ~_BV(XLAT_PIN);
    // Another 187.5 ns safely exceeding the minimum 300ns BLANK requirement
    __asm__("nop\n\t""nop\n\t""nop\n\t");
    BLANK_PORT &= ~_BV(BLANK_PIN);
    // -- End THE IMPORTANT BIT -- //
    
    PORTB &= ~_BV(PB1);
    
    if (++shiftRow == NUM_ROWS) {
      shiftRow = 0;
    }
    
    isShifting = 0;
  }
}

void setup() {
  // put your setup code here, to run once:
  XLAT_DDR |= _BV(XLAT_PIN);
  BLANK_DDR |= _BV(BLANK_PIN);
  
  DDRB |= _BV(PB1);
  
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  
  TlcMux_init();
}

int ledValues[7][21] = {
  {0,0,0,10,2024,512,0,1024,0,0,0,0,0,0,1024,0,0,0,2560,0,0}, 
  {1024,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1024,0}, 
  {1024,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1024,0}, 
  {1024,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1024,0}, 
  {1024,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1024,0}, 
  {1024,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1024,0}, 
  {1024,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1024,0},  
};
int direction[7][21] = {
  {0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0}, 
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0}, 
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, 
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, 
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, 
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, 
};

uint8_t color;
void loop()
{
//  TlcMux_set(1, 1, 4095);
//  for (uint8_t col = 0; col < 11; col++) {
//    for (uint8_t row = 0; row < NUM_ROWS; row++) {
//      TlcMux_set(row, col + color * 16, 4095);
//    }
//    color++;
//    if (color == 3) {
//      color = 0;
//    }
//  }
/*
  for (uint8_t row = 1; row < 7; row++) {
    for (uint8_t led = 1; led < 149; led += 6) {
       TlcMux_clear();
       TlcMux_set(row, led, 3095);

       if(led >= 3) {
        TlcMux_set(row, led - 3, 3095);
       }
       if(row >= 1) {
         TlcMux_set(row - 1, led, 3095);
       }
       if(led >= 3 && row >= 1) {
         TlcMux_set(row - 1, led - 3, 3095);
       }
       if(led >= 21) {
         TlcMux_set(row, led - 21, 3095);
       }
       if(led >= 21 && row >= 1) {
         TlcMux_set(row - 1, led - 21, 3095);
       }
       if(led >= 24) {
         TlcMux_set(row, led - 24, 3095);
       }
        if(led >= 24 && row >= 1) {
         TlcMux_set(row - 1, led - 24, 3095);
       }
       
    }
  }
  */
  
  for (uint8_t depth = 0; depth < 21; depth ++ ) {
    if (depth % 1 == 0) { delay(3000); TlcMux_clear(); }
    for (int intensity = 0; intensity < 2048; intensity += 64) {
      for (uint8_t led = depth; led < 149; led += 21) {
        for (uint8_t row = 0; row < 7; row++) {
          TlcMux_set(row, led, intensity);
        }
      }
    delay(20);
    }
  }
  
//  for (uint8_t led = 2; led < 149; led += 3) {
//    if(led % 21 == 2) { 
//      delay(4000);
//      TlcMux_clear();
//      
//    }
//    for (uint8_t row = 0; row < 7; row++) {
//      TlcMux_set(row, led, 3095);
//    }
//    delay(150);
//  }

/*
  for (uint8_t row = 0; row < NUM_ROWS; row++ ) {
    for (uint8_t led = 0; led < 21; led++) {
        ledValues[row][led] += direction[row][led] * 10;
        if(ledValues[row][led] >= 3085 || ledValues[row][led] <= 10) {
          direction[row][led] = -1 * direction[row][led];
        }
        
      TlcMux_set(row, led, ledValues[row][led]);
    }
  }
  delay(5);
  */
}

boolean isInBounds(int row, int cellIndex) {
  return (row >= 0 && row < NUM_ROWS && cellIndex >= 0 && cellIndex < 149);
}
