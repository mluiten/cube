/* Game of Life with multiplexing on a 3D RGB cube */

#define NUM_TLCS  10
#define NUM_ROWS  7

#define TLC_PWM_PERIOD 8192
#define TLC_GSCLK_PERIOD  3

#include "Tlc5940Mux.h"

#define  PULSE_PIN(port, pin)  port |= (1 << (pin)); port &= ~(1 << (pin))

#define DIMENSION   7
#define PLANE_SIZE  49
#define CUBE_SIZE   343

// True when using Moore, otherwise use Von Neumann
#define MOORE_NEIGHBORHOOD true

//Pin connected to latch pin (ST_CP) of 74HC595
#define LATCH_PIN 7
//Pin connected to clock pin (SH_CP) of 74HC595
#define CLOCK_PIN 8
////Pin connected to Data in (DS) of 74HC595
#define DATA_PIN 6

#define BIRTH_THRESHOLD_MIN 3
#define BIRTH_THRESHOLD_MAX 4
#define SURVIVE_THRESHOLD_MIN 2
#define SURVIVE_THRESHOLD_MAX 3

#define RED_CORRECTION 1.0
#define GREEN_CORRECTION 0.55
#define BLUE_CORRECTION 1.0

volatile uint8_t isShifting;
uint8_t shiftRow = 0;

/* This is our cell info. We can't use a CUBE_SIZE array, since the RAM on the Arduino runs out. So
 instead we use 8-bit integers and use bit-operations to indicate dead (0) or alive (1) */
byte *CELLS = (byte*) malloc(PLANE_SIZE * sizeof(byte));
byte *BACK_CELLS = (byte*) malloc(PLANE_SIZE * sizeof(byte));

#ifdef MOORE_NEIGHBORHOOD
#define NEIGHBORHOOD_SIZE 26
#elif
#define NEIGHBORHOOD_SIZE 6
#endif

ISR(TIMER1_OVF_vect)
{
  if (!isShifting) {
    isShifting = 1;
    PORTB |= _BV(PB1);

    TlcMux_shiftRow(shiftRow);

    // Keep filling the shift register with HIGH except for once every cycle
    if(shiftRow == 6) {
      PORTD &= ~_BV(PD6);
    } 
    else {
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
  randomSeed (analogRead (0));

  initiate_board();
  memcpy(CELLS, BACK_CELLS, PLANE_SIZE * sizeof(byte));

  // put your setup code here, to run once:
  XLAT_DDR |= _BV(XLAT_PIN);
  BLANK_DDR |= _BV(BLANK_PIN);

  DDRB |= _BV(PB1);

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);  
  pinMode(CLOCK_PIN, OUTPUT);

  TlcMux_init();
}

/**
 * Loops through the game of life cycle.
 *
 * 0. Update the cube according to the front buffer
 * 1. Copy the current state of the game to the back-buffer (where we make changes)
 * 2. Run a game-of-life gametick on the entire cube, one node at a time
 * 3. Switch the front and back-buffer
 */
uint16_t iteration_count = 0;
uint8_t colors[2] = {0, 1};
void loop() {
  int alive_nodes = 0;
  for (uint8_t row = 0; row < 7; row++) {
    TlcMux_clearRow(row);
    for (uint8_t cell = 0; cell < 49; cell++) {
      if(get_node_value((row * PLANE_SIZE) + cell)) {
        if(get_back_node_value((row * PLANE_SIZE) + cell)) {
          TlcMux_set(row, (cell * 3) + colors[1], 4095);
        } else {
          TlcMux_set(row, (cell * 3) + colors[0], 4095);
        }
        
        alive_nodes++;
      }

      //      TlcMux_clear();
      //      int *neigh = neighbors((row * PLANE_SIZE) + cell);
      //      TlcMux_set(row, (cell * 3) + 1, 3085);
      //      for (uint8_t i = 0; i < NEIGHBORHOOD_SIZE; i++) {
      //        if(neigh[i] >= 0) {
      //          TlcMux_set(int(neigh[i] / PLANE_SIZE), int(neigh[i] % PLANE_SIZE) * 3, 3095);
      //        }
      //      }
      //      free(neigh);
      //      delay(1000);
    }
  }

  //  int row = 3; int cell = 24;
  //  TlcMux_clear();
  //  int *neigh = neighbors(int(row * PLANE_SIZE) + cell);
  //  TlcMux_set(row, (cell * 3) + 1, 3085);
  //  for (uint8_t i = 0; i < NEIGHBORHOOD_SIZE; i++) {
  //    if(neigh[i] >= 0) {
  //      TlcMux_set(int(neigh[i] / PLANE_SIZE), int(neigh[i] % PLANE_SIZE) * 3, 3095);
  //    }
  //  }
  //  free(neigh);
  //  delay(1000);

  memcpy(BACK_CELLS, CELLS, PLANE_SIZE * sizeof(byte));
  for (int cellIndex = 0; cellIndex < CUBE_SIZE; cellIndex++) {
    gameOfLifeTick(cellIndex);
  }

  if (iteration_count++ >= 90 || alive_nodes <= 0) {
    initiate_board();
    iteration_count = 0;
    
    for(uint8_t c = 0; c < 2; c++) {
      colors[c] ++;
      
      if (colors[c] > 2) {
        colors[c] = 0;
      }
    }
  }

  byte *tmp = CELLS;
  CELLS = BACK_CELLS;
  BACK_CELLS = tmp;

  delay(100);
}

void initiate_board() {
  for (int i = 0; i < PLANE_SIZE; i++) {
    BACK_CELLS[i] = 0;
  }

  //int base = random(7,336);
  int base = 171;
  reanimate_cell(base - DIMENSION);
  reanimate_cell(base);
  reanimate_cell(base + DIMENSION);
}

void kill_cell(int cellIndex) {
  set_node_value(cellIndex, false);
}

void reanimate_cell(int cellIndex) {
  set_node_value(cellIndex, true);
}

void reanimate_random() {
  set_node_value(random(0, CUBE_SIZE), true);
}

/**
 * Executes one game-of-life step on the node by position cellIndex
 */
void gameOfLifeTick(int cellIndex) 
{ 
  if(get_node_value(cellIndex)) {
    life_tick_alive(cellIndex);
  } else {
    life_tick_dead(cellIndex);
  }
}

/** Actual GoL game ticks, depending on cell type */
void life_tick_dead(int cellIndex) {
  uint8_t living_neighbors = neighbors_alive(cellIndex);
  if (living_neighbors >= BIRTH_THRESHOLD_MIN && living_neighbors <= BIRTH_THRESHOLD_MAX) {
    reanimate_cell(cellIndex);
  }
}

void life_tick_alive(int cellIndex) {
  uint8_t living_neighbors = neighbors_alive(cellIndex);
  if (living_neighbors < SURVIVE_THRESHOLD_MIN || living_neighbors > SURVIVE_THRESHOLD_MAX) {
    kill_cell(cellIndex);
  }
}
/** End of GoL game ticks */

/** GoL checking functions */
boolean is_alive(int pos) {
  return get_node_value(pos);
}

uint8_t neighbors_alive(int pos) {
  /** TODO: Check if these are valid neighbors */
  int *neigh = neighbors(pos);
  uint8_t alive = 0;

  for (uint8_t i = 0; i < NEIGHBORHOOD_SIZE; i++) {
    if (neigh[i] >= 0 && is_alive(neigh[i])) {
      alive++;
    }
  }

  free(neigh);
  return alive;
}

/**
 * Returns the indexes of the cell's neighbors
 *
 * Currently uses the von Neumann neighborhood. Also checks for bounds of the cube's 
 * dimensions. Which involves a whole lot of range checking.
 *
 * @param int absolute position of the current cell
 * @return array of integers consisting of cell indexes
 */
int *neighbors(int cell_position) {
  int *neighbor_positions = (int*) malloc(NEIGHBORHOOD_SIZE * sizeof(int));

  for (int i = 0; i < NEIGHBORHOOD_SIZE; i++) {
    neighbor_positions[i] = -1;
  }

  // Checks for y-1; should be on the grid and in same column
  if (isInSameColumn(cell_position, - 1)) {
    neighbor_positions[0] = (cell_position - 1);
  }

  // Checks for y+1; should be on the grid and in same column
  if (isInSameColumn(cell_position, 1)) {
    neighbor_positions[1] = (cell_position + 1);
  }

  // Checks for x-1; should be on the grid and in same column
  if (isInSameRow(cell_position, - DIMENSION) && isInSamePlane(cell_position, - DIMENSION)) {
    neighbor_positions[2] = (cell_position - DIMENSION);
  }

  // Checks for x+1; should be on the grid and in same column
  if (isInSameRow(cell_position, DIMENSION) && isInSamePlane(cell_position, DIMENSION)) {
    neighbor_positions[3] = (cell_position + DIMENSION);
  }

  // Checks for z-1
  if (isInSameDepth(cell_position, - PLANE_SIZE)) {
    neighbor_positions[4] = (cell_position - PLANE_SIZE);
  }

  // Checks for z+1
  if (isInSameDepth(cell_position, PLANE_SIZE)) {
    neighbor_positions[5] = (cell_position + PLANE_SIZE);
  }

  // Check for outer nodes on the middle plane
  // TODO: This really should be done more structurally.
  if (MOORE_NEIGHBORHOOD) {
    if (neighbor_positions[0] >= 0 && neighbor_positions[2] >= 0)
      neighbor_positions[6] = cell_position - 1 - DIMENSION;
    if (neighbor_positions[0] >= 0 && neighbor_positions[3] >= 0)
      neighbor_positions[8] = cell_position - 1 + DIMENSION;
    if (neighbor_positions[1] >= 0 && neighbor_positions[2] >= 0)
      neighbor_positions[9] = cell_position + 1 - DIMENSION;
    if (neighbor_positions[1] >= 0 && neighbor_positions[3] >= 0)
      neighbor_positions[7] = cell_position + 1 + DIMENSION;   

    // Check out top,bottom,left,right nodes on front plane
    if (neighbor_positions[0] >= 0 && neighbor_positions[4] >= 0)
      neighbor_positions[10] = cell_position - 1 - PLANE_SIZE;
    if (neighbor_positions[0] >= 0 && neighbor_positions[5] >= 0)
      neighbor_positions[11] = cell_position - 1 + PLANE_SIZE;   
    if (neighbor_positions[1] >= 0 && neighbor_positions[4] >= 0)
      neighbor_positions[12] = cell_position + 1 - PLANE_SIZE;
    if (neighbor_positions[1] >= 0 && neighbor_positions[5] >= 0)
      neighbor_positions[13] = cell_position + 1 + PLANE_SIZE;

    // Check out top,bottom,left,right nodes on back plane
    if (neighbor_positions[2] >= 0 && neighbor_positions[4] >= 0)
      neighbor_positions[14] = cell_position - DIMENSION - PLANE_SIZE;
    if (neighbor_positions[2] >= 0 && neighbor_positions[5] >= 0)
      neighbor_positions[15] = cell_position - DIMENSION + PLANE_SIZE;   
    if (neighbor_positions[3] >= 0 && neighbor_positions[4] >= 0)
      neighbor_positions[16] = cell_position + DIMENSION - PLANE_SIZE;
    if (neighbor_positions[3] >= 0 && neighbor_positions[5] >= 0)
      neighbor_positions[17] = cell_position + DIMENSION + PLANE_SIZE;

    // Check out outer nodes on front and back plane
    if (neighbor_positions[2] >= 0 && neighbor_positions[4] >= 0 && neighbor_positions[0] >= 0)
      neighbor_positions[18] = cell_position - 1 - DIMENSION - PLANE_SIZE;
    if (neighbor_positions[2] >= 0 && neighbor_positions[4] >= 0 && neighbor_positions[1] >= 0)
      neighbor_positions[19] = cell_position + 1 - DIMENSION - PLANE_SIZE;   
    if (neighbor_positions[2] >= 0 && neighbor_positions[5] >= 0 && neighbor_positions[0] >= 0)
      neighbor_positions[20] = cell_position - 1 - DIMENSION + PLANE_SIZE;
    if (neighbor_positions[2] >= 0 && neighbor_positions[5] >= 0 && neighbor_positions[1] >= 0)
      neighbor_positions[21] = cell_position + 1 - DIMENSION + PLANE_SIZE;

    if (neighbor_positions[3] >= 0 && neighbor_positions[4] >= 0 && neighbor_positions[0] >= 0)
      neighbor_positions[22] = cell_position - 1 + DIMENSION - PLANE_SIZE;
    if (neighbor_positions[3] >= 0 && neighbor_positions[4] >= 0 && neighbor_positions[1] >= 0)
      neighbor_positions[23] = cell_position + 1 + DIMENSION - PLANE_SIZE;   
    if (neighbor_positions[3] >= 0 && neighbor_positions[5] >= 0 && neighbor_positions[0] >= 0)
      neighbor_positions[24] = cell_position - 1 + DIMENSION + PLANE_SIZE;
    if (neighbor_positions[3] >= 0 && neighbor_positions[5] >= 0 && neighbor_positions[1] >= 0)
      neighbor_positions[25] = cell_position + 1 + DIMENSION + PLANE_SIZE;

  }

  return neighbor_positions;
}

boolean isInSameColumn(int cell, int offset) {
  int new_cell = cell + offset;
  return (isInBounds(new_cell)
    && int(new_cell / DIMENSION) == int(cell / DIMENSION));
}

boolean isInSameRow(int cell, int offset) {
  int new_cell = cell + offset;
  return (isInBounds(new_cell)
    && int(new_cell % DIMENSION) == int(cell % DIMENSION));
}

boolean isInSamePlane(int cell, int offset) {
  int new_cell = cell + offset;
  return (isInBounds(new_cell)
    && int(new_cell / PLANE_SIZE) == int(cell / PLANE_SIZE));
}

boolean isInSameDepth(int cell, int offset) {
  int new_cell = cell + offset;
  return (isInBounds(new_cell)
    && int(new_cell % PLANE_SIZE) == int(cell % PLANE_SIZE));
}

boolean isInBounds(int cell) {
  return (cell >= 0 && cell < CUBE_SIZE); 
}

boolean get_node_value(int index) {
  return (CELLS[int(index / 7)] & bit(index % 7));
}

boolean get_back_node_value(int index) {
  return (BACK_CELLS[int(index / 7)] & bit(index % 7));
}

void set_node_value(int index, boolean value) {
  if (value)
    BACK_CELLS[int(index / 7)] |= bit(index % 7);
  else
    BACK_CELLS[int(index / 7)] &= 255 - bit(index % 7);
}

