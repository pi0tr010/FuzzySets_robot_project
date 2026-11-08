/*
 * What the robot program assumes about its own hardware.
 * The simulator has its own "true" values in sim/hardware.py; keep them in
 * sync, or change one on purpose to see how the algorithm copes with a
 * badly calibrated robot.
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#define ROBOT_WHEEL_DIAMETER_MM 65
#define ROBOT_TRACK_MM 130            /* distance between wheel centers */

/* DC 1:48 motor: 200 rpm @ 6 V, about 5.7 V after the TB6612FNG drop. */
#define ROBOT_MOTOR_RPM_FULL_PWM 190

/* SG90: 0.1 s / 60 deg @ 4.8 V. */
#define SERVO_DEG_PER_S 600
#define SERVO_SETTLE_MS 20
#define SERVO_CENTER_DEG 90

/* HC-SR04 recommended measurement cycle. */
#define SONAR_MIN_CYCLE_MS 60

#endif
