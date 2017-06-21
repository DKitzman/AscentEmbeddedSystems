#ifndef QDECF_H
#define QDECF_H

#define CW_DIR   1
#define CCW_DIR  0

void qdec_init(uint16_t cpr0, uint16_t cpr1);

//From testing: If qdecf_velN() returns negative, qdecf_dirN() returns true
uint16_t qdec_pos0();
float qdec_vel0();
uint8_t qdec_dir0();

uint16_t qdec_pos1();
float qdec_vel1();
uint8_t qdec_dir1();


#endif