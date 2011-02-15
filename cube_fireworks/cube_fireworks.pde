#define  NUM_TLCS  10
#define  NUM_ROWS  7

#define NUM_ARROWS 2
#define MAX_PARTICLES 12

#define TLC_PWM_PERIOD 8192
#define TLC_GSCLK_PERIOD  3

#include "Tlc5940Mux.h"
#include "firework_structs.h"

#define  PULSE_PIN(port, pin)  port |= (1 << (pin)); port &= ~(1 << (pin))

const int latchPin = 7;
//Pin connected to clock pin (SH_CP) of 74HC595
const int clockPin = 8;
////Pin connected to Data in (DS) of 74HC595
const int dataPin = 6;

volatile uint8_t isShifting;
uint8_t shiftRow = 0;

Arrow arrows[NUM_ARROWS];

/**
 * Multiplexing function: DO NOT TOUCH
 */
ISR(TIMER1_OVF_vect)
{
  if (!isShifting) {
    isShifting = 1;
    PORTB |= _BV(PB1);
    
    TlcMux_shiftRow(shiftRow);
  
    // Keep filling the register with HIGH except for once every cycle
    if(shiftRow == 6) {
      PORTD &= ~_BV(PD6);
    } else {
      PORTD |= _BV(PD6);
    }
    PULSE_PIN(PORTD, PD7);
    PULSE_PIN(PORTB, PB0);
    
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
  randomSeed(analogRead(0));
  
  // put your setup code here, to run once:
  XLAT_DDR |= _BV(XLAT_PIN);
  BLANK_DDR |= _BV(BLANK_PIN);
  
  DDRB |= _BV(PB1);
  
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  
  TlcMux_init();
}


Color_t arrowColor = Color_t(255, 0, 0);
Color_t particleColor = Color_t(0, 255, 0);

int ActiveParticles[NUM_ARROWS] = {0};  
void loop()
{
  randomSeed(analogRead(0));
  TlcMux_clear();
  
  for (int a = 0; a < NUM_ARROWS; a++) {
    if (arrows[a].isFired) {
      if (arrows[a].Position.y < arrows[a].ExplodeHeight) {
        // Draw the arrow that is traveling
        paint(
          Coordinate_t(
            arrows[a].Position.x,
            arrows[a].Position.y - 1, 
            arrows[a].Position.z
          ),
          Color_t(
            arrowColor.Red / 6,
            arrowColor.Green / 6,
            arrowColor.Blue / 6
          )
        );
        paint(arrows[a].Position, arrowColor);
        arrows[a].Position.y++;
      } else {
        paint(
          arrows[a].Position, 
          Color_t(
            255 / (ActiveParticles[a] * 10),
            255 / (ActiveParticles[a] * 2),
            255 / (ActiveParticles[a] * 10)
          )
        );
        
        // Exploooode                
        for (int i = 0; i < MAX_PARTICLES; i++) {
          paint(
            Coordinate_t(
              arrows[a].Particles[i].Position.x,
              arrows[a].Particles[i].Position.y - 1, 
              arrows[a].Particles[i].Position.z
            ),
            Color_t(
              arrows[a].Particles[i].Color.Red / 12 / (ActiveParticles[a] * 2),
              arrows[a].Particles[i].Color.Green / 12 / (ActiveParticles[a] * 2),
              arrows[a].Particles[i].Color.Blue / 12 / (ActiveParticles[a] * 2)
            )
          );
        
          paint(
            arrows[a].Particles[i].Position, 
            Color_t(
              arrows[a].Particles[i].Color.Red / 6 / (ActiveParticles[a] * 2),
              arrows[a].Particles[i].Color.Green / 6 / (ActiveParticles[a] * 2),
              arrows[a].Particles[i].Color.Blue / 6 / (ActiveParticles[a] * 2)
            )
          );
   
          arrows[a].Particles[i].Position.x += arrows[a].Particles[i].Direction.x;
          arrows[a].Particles[i].Position.y += arrows[a].Particles[i].Direction.y;
          arrows[a].Particles[i].Position.z += arrows[a].Particles[i].Direction.z;
          
          // Looks like gravity
          if (arrows[a].Particles[i].Direction.y >= 0 && random(0,3) == 0) {
            arrows[a].Particles[i].Direction.y--;
            
            // Looks like air resistance
            if (arrows[a].Particles[i].Direction.x > 0 && random(0,3) == 0) {
              arrows[a].Particles[i].Direction.x--;
            }
            if (arrows[a].Particles[i].Direction.z > 0 && random(0,3) == 0) {
              arrows[a].Particles[i].Direction.z--;
            }
          }          
          
          paint(arrows[a].Particles[i].Position,
            Color_t(
              arrows[a].Particles[i].Color.Red / (ActiveParticles[a] * 2),
              arrows[a].Particles[i].Color.Green / (ActiveParticles[a] * 2),
              arrows[a].Particles[i].Color.Blue / (ActiveParticles[a] * 2)
            )
          );
        }
        
        if(ActiveParticles[a]++ > 5) { 
          arrows[a].isFired = false;
          free(arrows[a].Particles);
          ActiveParticles[a] = 0;
        }
      }
    } else if(random(0, 6) == 0) {
      Particle *particles = (Particle*) malloc(MAX_PARTICLES * sizeof(Particle));
  
      arrows[a].Position.x = random(0,7);
      arrows[a].Position.z = random(0,7);
      arrows[a].Position.y = 0;
      
      arrows[a].ExplodeHeight = random(3,7);
      arrows[a].Particles = particles;
      arrows[a].isFired = true;
      
      for (int i = 0; i < MAX_PARTICLES; i++) {        
        particles[i].Position.x = arrows[a].Position.x;
        particles[i].Position.y = arrows[a].ExplodeHeight;
        particles[i].Position.z = arrows[a].Position.z;
        
        particles[i].Direction.x = (random(0,3) - 1);
        particles[i].Direction.y = (random(0,3) - 1);
        particles[i].Direction.z = (random(0,3) - 1);
        
        particles[i].Color.Red = random(0, 192);
        particles[i].Color.Green = random(0, 192);
        particles[i].Color.Blue = random(0,192);
      }
    }
  }
  
  delay(120);
}

void paint(struct Coordinate_t coord, Color_t color) {
  if(!isInBounds(coord))
    return;
  
  TlcMux_set(coord.z, (coord.x * NUM_ROWS * 3) + (coord.y * 3), (int) ((color.Red / 255.0) * 4095));
  TlcMux_set(coord.z, (coord.x * NUM_ROWS * 3) + (coord.y * 3) + 1, (int) ((color.Green / 255.0) * 4095));
  TlcMux_set(coord.z, (coord.x * NUM_ROWS * 3) + (coord.y * 3) + 2, (int) ((color.Blue / 255.0) * 4095));
}

boolean isInBounds(struct Coordinate_t part) {
  if (part.x < 0 || part.x >= NUM_ROWS
      || part.y < 0 || part.y >= NUM_ROWS
      || part.z < 0 || part.z >= NUM_ROWS
  ) {
    return false;
  }
  
  return true;
}
