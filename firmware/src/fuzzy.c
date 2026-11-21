/*
 * 0-order TSK fuzzy correction for right-hand wall following.
 * Inputs: right distance, angle to the right wall, front distance.
 * Output: steer, speed.
 * Final = sum(wi*zi) / sum(wi)  (Instrukcja 4).
 */
#include "fuzzy.h"

static float clampf01(float x)
{
    if (x < 0.0f)
        return 0.0f;
    if (x > 1.0f)
        return 1.0f;
    return x;
}

static float tri(float x, float a, float b, float c)
{
    if (x <= a || x >= c)
        return 0.0f;
    if (x < b)
        return (x - a) / (b - a);
    return (c - x) / (c - b);
}

static float trap(float x, float a, float b, float c, float d)
{
    if (x <= a || x >= d)
        return 0.0f;
    if (x >= b && x <= c)
        return 1.0f;
    if (x < b)
        return (x - a) / (b - a);
    return (d - x) / (d - c);
}

static float fmin2(float a, float b)
{
    return a < b ? a : b;
}

typedef struct {
    float w, steer, speed;
} Acc;

static void fire(Acc *a, float w, float steer, float speed)
{
    if (w <= 0.0f)
        return;
    a->w += w;
    a->steer += w * steer;
    a->speed += w * speed;
}

void fuzzy_wall_follow(uint16_t right_cm, uint16_t front_cm, uint16_t follow_cm,
                       float angle_deg, FuzzyOut *out)
{
    float err = (float)right_cm - (float)follow_cm; /* + = too far from wall */
    float F = (float)front_cm;

    /* Error labels (cm). */
    float e_neg = trap(err, -40.0f, -40.0f, -8.0f, -2.0f);   /* too close */
    float e_ok  = tri(err, -5.0f, 0.0f, 5.0f);
    float e_pos = trap(err, 2.0f, 8.0f, 40.0f, 40.0f);        /* too far */

    /* Angle labels (deg). */
    float a_tow = trap(angle_deg, -90.0f, -90.0f, -12.0f, -2.0f); /* toward wall */
    float a_par = tri(angle_deg, -8.0f, 0.0f, 8.0f);
    float a_awy = trap(angle_deg, 2.0f, 12.0f, 90.0f, 90.0f);     /* away */

    float f_near = trap(F, 0.0f, 0.0f, 22.0f, 36.0f);
    float f_ok   = trap(F, 28.0f, 40.0f, 200.0f, 200.0f);

    Acc a = {0.0f, 0.0f, 0.0f};

    /*
     * Rule table (steer, + = left):
     *              toward   parallel   away
     *   too close   +0.9      +0.5      0.0
     *   ok          +0.5       0.0     -0.5
     *   too far      0.0      -0.5     -0.9
     */
    fire(&a, fmin2(fmin2(e_neg, a_tow), f_ok), 0.90f, 0.50f);
    fire(&a, fmin2(fmin2(e_neg, a_par), f_ok), 0.50f, 0.80f);
    fire(&a, fmin2(fmin2(e_neg, a_awy), f_ok), 0.00f, 1.00f);
    fire(&a, fmin2(fmin2(e_ok, a_tow), f_ok), 0.50f, 0.80f);
    fire(&a, fmin2(fmin2(e_ok, a_par), f_ok), 0.00f, 1.00f);
    fire(&a, fmin2(fmin2(e_ok, a_awy), f_ok), -0.50f, 0.80f);
    fire(&a, fmin2(fmin2(e_pos, a_tow), f_ok), 0.00f, 1.00f);
    fire(&a, fmin2(fmin2(e_pos, a_par), f_ok), -0.50f, 0.80f);
    fire(&a, fmin2(fmin2(e_pos, a_awy), f_ok), -0.90f, 0.60f);

    /* Front getting tight → slow down (turn handled outside). */
    fire(&a, f_near, 0.20f, 0.25f);

    if (a.w < 1e-4f) {
        out->steer = 0.0f;
        out->speed = 0.6f;
        return;
    }

    {
        float s = a.steer / a.w;

        if (s > 1.0f)
            s = 1.0f;
        if (s < -1.0f)
            s = -1.0f;
        out->steer = s;
        out->speed = clampf01(a.speed / a.w);
    }
}
