#ifndef LED_H
#define LED_H

void led_init();

void led_cmdOn();
void led_cmdOff();
void led_cmdToggle();

void led_pingOn();
void led_pingOff();
void led_pingToggle();

void led_aliveOn();
void led_aliveOff();
void led_aliveToggle();

void led_dataOut(uint8_t value);
void led_bitSet(uint8_t bit);
void led_bitClear(uint8_t bit);
void led_bitToggle(uint8_t bit);

#endif