#ifndef NOU3_ENCODER_H
#define NOU3_ENCODER_H

#include <Arduino.h>

#define MAX_ENCODERS 8

class NoU_Encoder {
public:
    NoU_Encoder();
    void begin(uint8_t pinA, uint8_t pinB);
    int32_t getPosition();
    void update();

private:
    uint8_t _pinA, _pinB;
    volatile int32_t _position;
    volatile uint8_t _prevState;
    uint8_t _index;

    // Individual ISR functions
    static void isr0();
    static void isr1();
    static void isr2();
    static void isr3();
    static void isr4();
    static void isr5();
    static void isr6();
    static void isr7();

    static uint8_t numEncoders;
    static NoU_Encoder* instances[MAX_ENCODERS];
};

#endif // NOU3_ENCODER_H
