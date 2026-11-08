/*
 * Arduino Uno wiring. Servo.h takes Timer1, so pins 9 and 10 lose PWM:
 * motor PWM uses 5 and 6, the servo signal goes to 10.
 */
#ifndef HAL_ARDUINO_PINS_H
#define HAL_ARDUINO_PINS_H

/* TB6612FNG: channel A = left motor, channel B = right motor */
#define PIN_MOTOR_A_PWM 5
#define PIN_MOTOR_A_IN1 7
#define PIN_MOTOR_A_IN2 8
#define PIN_MOTOR_B_PWM 6
#define PIN_MOTOR_B_IN1 4
#define PIN_MOTOR_B_IN2 2
#define PIN_MOTOR_STBY 3

/* Flip if a wheel spins backwards on positive PWM. */
#define MOTOR_LEFT_INVERT false
#define MOTOR_RIGHT_INVERT false

#define PIN_SERVO 10

#define PIN_SONAR_TRIG 12
#define PIN_SONAR_ECHO 11

#endif
