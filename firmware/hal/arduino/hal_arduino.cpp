/*
 * HAL for the real robot: Arduino Uno + TB6612FNG + SG90 + HC-SR04.
 * Not tested on hardware yet. Pins are in hal_arduino_pins.h.
 */
#include <Arduino.h>
#include <Servo.h>
#include <stdarg.h>
#include <stdio.h>

#include "hal.h"
#include "hal_arduino_pins.h"

/* Echo pulse for ~650 cm; longer means no echo (sensor reports ~38 ms). */
#define SONAR_TIMEOUT_US 38000UL

static Servo servo;

static void motor(uint8_t pwm_pin, uint8_t in1, uint8_t in2, int16_t v, bool invert)
{
    if (invert)
        v = -v;
    if (v > 255)
        v = 255;
    if (v < -255)
        v = -255;
    digitalWrite(in1, v > 0 ? HIGH : LOW);
    digitalWrite(in2, v < 0 ? HIGH : LOW);
    analogWrite(pwm_pin, v < 0 ? -v : v);
}

void hal_init(void)
{
    Serial.begin(115200);
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    pinMode(PIN_MOTOR_A_IN1, OUTPUT);
    pinMode(PIN_MOTOR_A_IN2, OUTPUT);
    pinMode(PIN_MOTOR_A_PWM, OUTPUT);
    pinMode(PIN_MOTOR_B_IN1, OUTPUT);
    pinMode(PIN_MOTOR_B_IN2, OUTPUT);
    pinMode(PIN_MOTOR_B_PWM, OUTPUT);
    digitalWrite(PIN_MOTOR_STBY, HIGH);
    hal_motors(0, 0);

    pinMode(PIN_SONAR_TRIG, OUTPUT);
    pinMode(PIN_SONAR_ECHO, INPUT);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    servo.attach(PIN_SERVO);
    servo.write(90);
}

uint32_t hal_millis(void) { return millis(); }
uint32_t hal_micros(void) { return micros(); }
void hal_delay_ms(uint32_t ms) { delay(ms); }

void hal_delay_us(uint32_t us)
{
    /* delayMicroseconds() is only accurate up to 16383 us */
    while (us > 16000UL) {
        delayMicroseconds(16000);
        us -= 16000UL;
    }
    delayMicroseconds((unsigned int)us);
}

void hal_motors(int16_t left, int16_t right)
{
    motor(PIN_MOTOR_A_PWM, PIN_MOTOR_A_IN1, PIN_MOTOR_A_IN2, left, MOTOR_LEFT_INVERT);
    motor(PIN_MOTOR_B_PWM, PIN_MOTOR_B_IN1, PIN_MOTOR_B_IN2, right, MOTOR_RIGHT_INVERT);
}

void hal_servo_write(uint8_t angle_deg)
{
    servo.write(angle_deg > 180 ? 180 : angle_deg);
}

uint32_t hal_sonar_echo_us(void)
{
    digitalWrite(PIN_SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONAR_TRIG, LOW);
    return pulseIn(PIN_SONAR_ECHO, HIGH, SONAR_TIMEOUT_US);
}

void hal_log(const char *fmt, ...)
{
    char buf[64];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    Serial.println(buf);
}
