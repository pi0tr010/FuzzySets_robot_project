/*
 * Fuzzy wall-hug (0-order TSK): distance to followed wall + front → steer.
 * Steer: +1 = left (CCW), -1 = right (CW). Speed: 0..1.
 */
#ifndef FUZZY_H
#define FUZZY_H

#include <stdint.h>

typedef struct {
    float steer;
    float speed;
} FuzzyOut;

/*
 * follow_cm = target side distance; right_cm / front_cm filtered distances.
 * angle_deg = heading relative to the right wall: + = turned away (left),
 * - = turned toward it, 0 = parallel or unknown.
 */
void fuzzy_wall_follow(uint16_t right_cm, uint16_t front_cm, uint16_t follow_cm,
                       float angle_deg, FuzzyOut *out);

#endif
