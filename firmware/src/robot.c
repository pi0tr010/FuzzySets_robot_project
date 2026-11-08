/*
 * Demo program: drive forward until a wall is close, look around with the
 * servo, turn towards the longest free distance. Only meant to exercise the
 * simulator; the real algorithm (fuzzy control + A*) replaces it later.
 */
#include "hal.h"
#include "robot.h"
#include "robot_config.h"

#define DRIVE_PWM 140
#define TURN_PWM 120
#define STOP_DISTANCE_CM 18
#define SCAN_STEP_DEG 30
#define NO_ECHO_CM 999

static uint8_t servo_pos = SERVO_CENTER_DEG;
static uint32_t last_ping_ms;

static void servo_move(uint8_t deg)
{
    uint8_t diff = deg > servo_pos ? (uint8_t)(deg - servo_pos) : (uint8_t)(servo_pos - deg);
    hal_servo_write(deg);
    hal_delay_ms((uint32_t)diff * 1000u / SERVO_DEG_PER_S + SERVO_SETTLE_MS);
    servo_pos = deg;
}

static uint16_t ping_cm(void)
{
    uint32_t since = hal_millis() - last_ping_ms;
    uint32_t us;

    if (since < SONAR_MIN_CYCLE_MS)
        hal_delay_ms(SONAR_MIN_CYCLE_MS - since);
    last_ping_ms = hal_millis();
    us = hal_sonar_echo_us();
    return us == 0 ? NO_ECHO_CM : (uint16_t)(us / HAL_SONAR_US_PER_CM);
}

/* deg > 0 turns left (counter-clockwise), open loop: no encoders. */
static void turn_in_place(int16_t deg)
{
    const float wheel_mm_s = ROBOT_MOTOR_RPM_FULL_PWM / 60.0f * 3.14159f
                             * ROBOT_WHEEL_DIAMETER_MM * TURN_PWM / 255.0f;
    const float deg_per_s = 2.0f * wheel_mm_s / ROBOT_TRACK_MM * 57.2958f;
    uint16_t mag = (uint16_t)(deg < 0 ? -deg : deg);

    if (mag == 0)
        return;
    if (deg > 0)
        hal_motors(-TURN_PWM, TURN_PWM);
    else
        hal_motors(TURN_PWM, -TURN_PWM);
    hal_delay_ms((uint32_t)(mag / deg_per_s * 1000.0f));
    hal_motors(0, 0);
    hal_delay_ms(50);
}

void robot_setup(void)
{
    hal_motors(0, 0);
    hal_servo_write(SERVO_CENTER_DEG);
    hal_delay_ms(500);
    hal_log("demo robot ready");
}

void robot_loop(void)
{
    uint16_t best_cm = 0;
    uint8_t best_deg = SERVO_CENTER_DEG;
    uint16_t a;
    uint16_t d = ping_cm();

    if (d > STOP_DISTANCE_CM) {
        hal_motors(DRIVE_PWM, DRIVE_PWM);
        return;
    }

    hal_motors(0, 0);
    hal_log("wall at %u cm, scanning", d);
    for (a = 0; a <= 180; a += SCAN_STEP_DEG) {
        uint16_t cm;
        servo_move((uint8_t)a);
        cm = ping_cm();
        if (cm > best_cm) {
            best_cm = cm;
            best_deg = (uint8_t)a;
        }
    }
    servo_move(SERVO_CENTER_DEG);

    if (best_cm <= STOP_DISTANCE_CM) {
        hal_log("dead end, turning back");
        turn_in_place(180);
    } else {
        hal_log("best %u cm at servo %u deg", best_cm, best_deg);
        turn_in_place((int16_t)best_deg - SERVO_CENTER_DEG);
    }
}
