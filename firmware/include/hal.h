/*
 * Hardware abstraction layer of the maze robot.
 *
 * Robot logic (src/) talks to the hardware ONLY through these functions.
 * Two implementations exist:
 *   hal/sim/hal_sim.c          - talks to the Python simulator over stdin/stdout
 *   hal/arduino/hal_arduino.cpp - real pins on Arduino Uno (TB6612FNG, SG90, HC-SR04)
 *
 * Target is ATmega328P (2 KB RAM, int is 16 bit): use stdint types, avoid
 * large stack buffers, and remember that printf on AVR does not format %f.
 */
#ifndef HAL_H
#define HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Called once by the platform before robot_setup(). */
void hal_init(void);

/* Time since power on. Wraps around like Arduino millis()/micros(). */
uint32_t hal_millis(void);
uint32_t hal_micros(void);

void hal_delay_ms(uint32_t ms);
void hal_delay_us(uint32_t us);

/*
 * Motor PWM through TB6612FNG, range -255..255 (sign = direction).
 * left/right as seen from the robot looking forward.
 * Positive on both wheels drives forward.
 */
void hal_motors(int16_t left, int16_t right);

/*
 * SG90 servo target angle, 0..180 degrees. 90 = sensor looks forward,
 * 0 = right, 180 = left. The call returns immediately; the servo needs
 * time to get there (about 0.1 s / 60 deg).
 */
void hal_servo_write(uint8_t angle_deg);

/*
 * Fires HC-SR04 and returns the echo pulse width in microseconds
 * (blocks for the whole measurement). 0 = no echo (timeout).
 * Keep at least 60 ms between pings.
 */
uint32_t hal_sonar_echo_us(void);

#define HAL_SONAR_US_PER_CM 58u

/* Debug text (Serial on Arduino, log panel in the simulator). */
void hal_log(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
