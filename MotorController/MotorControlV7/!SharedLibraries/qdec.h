#ifndef QDEC_H
#define QDEC_H

#define CW_DIR   1
#define CCW_DIR  0

void qdec_init(uint16_t cpr0, uint16_t cpr1);

//From testing: If qdec_velN() returns negative, qdec_dirN() returns true
uint16_t qdec_pos0();
int16_t qdec_vel0();
uint8_t qdec_dir0();

uint16_t qdec_pos1();
int16_t qdec_vel1();
uint8_t qdec_dir1();


#endif