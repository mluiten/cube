#ifndef __FIREWORK_STRUCTS__H__
#define __FIREWORK_STRUCTS__H__

#define byte uint8_t

struct Coordinate_t {
  Coordinate_t() {}
  Coordinate_t(int8_t x, int8_t y, int8_t z) : x(x), y(y), z(z) {}
  
  int8_t x;
  int8_t y;
  int8_t z;
};

struct Color_t {
  Color_t() {}
  Color_t(const Color_t &other) {Red = other.Red; Green = other.Green; Blue = other.Blue; }
  Color_t(byte r, byte g, byte b) : Red(r), Green(g), Blue(b) {}
  
  byte Red;
  byte Green;
  byte Blue;
};

typedef struct {  
  Color_t Color;
  
  Coordinate_t Position;
  Coordinate_t Direction;
} Particle;

typedef struct {
  bool isFired;
  Coordinate_t Position;
	
  byte ExplodeHeight;

  Particle *Particles;
} Arrow;

#endif
