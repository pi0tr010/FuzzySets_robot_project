/*
 * Right-hand wall follower (Striker-style) + fuzzy side-distance correction.
 * One HC-SR04 on a servo: while driving, scan 0..180° in 30° steps (front
 * prioritized). Echo misses are filtered. Forward only — no reverse.
 *
 * Junction rules (follow RIGHT wall):
 *   front wall     → turn LEFT  until front clears
 *   right wall gone → turn RIGHT until right wall returns
 *   else           → drive with fuzzy hug of followCm
 */
#include <math.h>

#include "hal.h"
#include "robot.h"
#include "robot_config.h"
#include "fuzzy.h"

#define DRIVE_PWM 50
#define TURN_PWM 20

/* Passage model (~Striker corridorWidthCm scaling). */
#define RANGE_CM 100
#define SIDE_WALL_CM 35   /* closer than this ⇒ side wall present */
#define FRONT_STOP_CM 18  /* closer than this ⇒ stop and turn */
#define FOLLOW_CM 15      /* target distance to right wall */
#define MISS_BEFORE_OPEN 2

#define SCAN_STEP 30
#define SCAN_BINS 7

#define ANG_FRONT 90
#define STUCK_TURN_LOOPS 28 /* ~stuck spinning → full escape scan */
#define STUCK_LEFT_TURNS 4  /* 4 × 90° left and front still blocked */
#define RIGHT_PRE_CM 27     /* wall end at sensor → axle to gap centre */
#define WALL_END_MAX_PINGS 40 /* ~2.4 s / 30 cm of looking for the wall end */
#define DRIVE_MIN_FRONT_CM 10 /* timed straight runs stop closer than this */
#define ALIGN_MAX_DEG 20.0f   /* larger "misalignment" = bad reading, ignore */
#define FOLLOW_STUCK_SAMPLES 8 /* unchanged front pings while driving = stuck */
#define RIGHT_POST_CM 25    /* forward after that turn (sensor past the corner) */

enum {
    MODE_FOLLOW = 0,
    MODE_TURN_LEFT = 1,
    MODE_TURN_RIGHT = 2
};

/* Prefer front samples while moving (bin indices into 0,30,...,180). */
static const uint8_t SCAN_ORDER[] = {
    3, 0, 1, 0, 3, 0, 1, 0, 3, 6, 3, 0, 1, 0, 3, 2, 3, 4
};
#define SCAN_ORDER_LEN (sizeof SCAN_ORDER / sizeof SCAN_ORDER[0])

static uint8_t servo_pos = ANG_FRONT;
static uint32_t last_ping_ms;

static uint16_t raw_bins[SCAN_BINS];
static uint16_t filt_l, filt_f, filt_r;
static uint8_t miss_l, miss_f, miss_r;

static uint8_t scan_ord_i;
static uint8_t mode;
static uint8_t seen_wall;
static int16_t pwm_l, pwm_r;
static uint8_t turn_loops; /* consecutive loops in a turn mode */
static uint16_t last_front_raw;
static uint8_t same_front;  /* front samples that did not change while driving */

static void servo_move(uint8_t deg)
{
    uint8_t diff = deg > servo_pos ? (uint8_t)(deg - servo_pos) : (uint8_t)(servo_pos - deg);

    hal_servo_write(deg);
    if (diff > 0)
        hal_delay_ms((uint32_t)diff * 1000u / SERVO_DEG_PER_S + SERVO_SETTLE_MS);
    servo_pos = deg;
}

static uint16_t ping_raw_cm(void)
{
    uint32_t since = hal_millis() - last_ping_ms;
    uint32_t us;
    uint16_t cm;

    if (since < SONAR_MIN_CYCLE_MS)
        hal_delay_ms(SONAR_MIN_CYCLE_MS - since);
    last_ping_ms = hal_millis();
    us = hal_sonar_echo_us();
    if (us == 0)
        return 0;
    cm = (uint16_t)(us / HAL_SONAR_US_PER_CM);
    return cm > RANGE_CM ? RANGE_CM : cm;
}

/* Striker-style: a few misses keep last value; then treat as open (RANGE). */
static uint16_t filter_dist(uint16_t raw, uint16_t *prev, uint8_t *misses)
{
    if (raw == 0) {
        (*misses)++;
        if (*prev > 0 && *prev < RANGE_CM && *misses < MISS_BEFORE_OPEN)
            return *prev;
        if (*misses >= MISS_BEFORE_OPEN) {
            *prev = RANGE_CM;
            return RANGE_CM;
        }
        return *prev;
    }
    *misses = 0;
    if (*prev == 0 || *prev >= RANGE_CM) {
        *prev = raw;
        return raw;
    }
    *prev = (uint16_t)((raw + *prev) / 2u);
    return *prev;
}

static void set_motors(int16_t left, int16_t right)
{
    if (left > 255)
        left = 255;
    if (right > 255)
        right = 255;
    if (left < -255)
        left = -255;
    if (right < -255)
        right = -255;
    pwm_l = left;
    pwm_r = right;
    hal_motors(pwm_l, pwm_r);
}

static void stop_motors(void)
{
    set_motors(0, 0);
}

static void spin_left(void)
{
    set_motors(-TURN_PWM, TURN_PWM);
}

static void spin_right(void)
{
    set_motors(TURN_PWM, -TURN_PWM);
}

/* Timed in-place yaw (deg > 0 = left / CCW). Used only by escape scan. */
static void turn_by_deg(int16_t deg)
{
    const float wheel_mm_s = ROBOT_MOTOR_RPM_FULL_PWM / 60.0f * 3.14159f
                             * ROBOT_WHEEL_DIAMETER_MM * TURN_PWM / 255.0f;
    const float deg_per_s = 2.0f * wheel_mm_s / ROBOT_TRACK_MM * 57.2958f;
    uint16_t mag = (uint16_t)(deg < 0 ? -deg : deg);

    if (mag == 0)
        return;
    if (deg > 0)
        spin_left();
    else
        spin_right();
    hal_delay_ms((uint32_t)(mag / deg_per_s * 1000.0f));
    stop_motors();
    hal_delay_ms(40);
}

/*
 * Stuck while turning: full 0..180 / 30° scan, pick freest direction,
 * but not toward the wall we were already turning into.
 * mode TURN_LEFT  → ignore front sector (60/90/120)
 * mode TURN_RIGHT → ignore right sector (0/30)
 */
static void escape_full_scan(void)
{
    uint8_t i;
    uint16_t best_cm = 0;
    uint8_t best_deg = ANG_FRONT;
    uint8_t forbid_right = (mode == MODE_TURN_RIGHT);
    uint8_t forbid_front = (mode == MODE_TURN_LEFT);
    int16_t yaw;

    stop_motors();
    hal_log("stuck -> full scan");

    for (i = 0; i < SCAN_BINS; i++) {
        uint8_t deg = (uint8_t)(i * SCAN_STEP);
        uint16_t cm;

        if (forbid_right && i <= 1)
            continue;
        if (forbid_front && i >= 2 && i <= 4)
            continue;

        servo_move(deg);
        cm = ping_raw_cm();
        raw_bins[i] = cm;
        if (cm == 0)
            continue;
        if (cm > best_cm || (cm + 5 >= best_cm && deg < best_deg)) {
            best_cm = cm;
            best_deg = deg;
        }
    }

    /* Fallback if everything forbidden / empty: opposite of stuck turn. */
    if (best_cm < FRONT_STOP_CM) {
        yaw = forbid_right ? 90 : -90;
        hal_log("scan empty, yaw %d", (int)yaw);
    } else {
        yaw = (int16_t)best_deg - ANG_FRONT;
        hal_log("scan best %u cm @ %u -> yaw %d", best_cm, best_deg, (int)yaw);
    }

    /* Never stay put after escape — front "best" would give yaw 0.
     * Stuck while following: away from the right wall (left). */
    if (yaw == 0)
        yaw = (mode == MODE_TURN_LEFT) ? -90 : 90;

    turn_by_deg(yaw);
    turn_loops = 0;
    mode = MODE_FOLLOW;
    servo_move(ANG_FRONT);
    set_motors(DRIVE_PWM, DRIVE_PWM);
}

static void update_sector_filters(uint8_t bin, uint16_t raw)
{
    raw_bins[bin] = raw;
    /* 30° beam: nearest ray is 22.5° off the side-wall normal. */
    if (bin == 1 && raw > 0)
        raw = (uint16_t)(raw * 0.924f);
    if (bin <= 1)
        filt_r = filter_dist(raw, &filt_r, &miss_r);
    else if (bin >= 2 && bin <= 4)
        filt_f = filter_dist(raw, &filt_f, &miss_f);
    else
        filt_l = filter_dist(raw, &filt_l, &miss_l);
}

/* Drive + one prioritized scan sample. */
static void scan_while_driving(void)
{
    uint8_t bin = SCAN_ORDER[scan_ord_i];
    uint8_t deg = (uint8_t)(bin * SCAN_STEP);
    uint16_t raw;

    if (pwm_l != 0 || pwm_r != 0)
        hal_motors(pwm_l, pwm_r);

    servo_move(deg);

    if (pwm_l != 0 || pwm_r != 0)
        hal_motors(pwm_l, pwm_r);

    raw = ping_raw_cm();
    update_sector_filters(bin, raw);

    /* No bump sensor: driving forward with a frozen front reading = stuck. */
    if (bin == 3) {
        if (raw > 0 && raw < RANGE_CM && (pwm_l > 0 || pwm_r > 0)
            && raw + 1 >= last_front_raw && raw <= last_front_raw + 1)
            same_front++;
        else
            same_front = 0;
        last_front_raw = raw;
    }

    scan_ord_i++;
    if (scan_ord_i >= SCAN_ORDER_LEN)
        scan_ord_i = 0;
}

/* During a turn, keep checking the relevant sector. */
static void ping_front_only(void)
{
    uint16_t raw;

    servo_move(ANG_FRONT);
    raw = ping_raw_cm();
    update_sector_filters(3, raw);
}

static void ping_right_only(void)
{
    uint16_t raw;

    servo_move(0);
    raw = ping_raw_cm();
    update_sector_filters(0, raw);
}

static int wall_right(void)
{
    return filt_r > 0 && filt_r < SIDE_WALL_CM;
}

static int wall_front(void)
{
    return filt_f > 0 && filt_f < FRONT_STOP_CM;
}

static int wall_left(void)
{
    return filt_l > 0 && filt_l < SIDE_WALL_CM;
}

/*
 * Heading vs a side wall from its perpendicular and 30°-forward samples
 * (+ = turned away from that wall). The sonar returns its nearest ray, so the
 * 30° sample really measures along the beam edge, 22.5° off the normal:
 * d0 = D/cos(a), d1 = D/cos(a+22.5°)  =>  tan(a) = (cos22.5° - d0/d1)/sin22.5°.
 * Distances measured from the servo axis (sensor sits 1.5 cm in front of it).
 */
static float wall_angle_from(uint16_t perp, uint16_t diag)
{
    float d0 = perp + 1.5f, d1 = diag + 1.5f;

    return atanf((0.924f - d0 / d1) / 0.383f) * 57.2958f;
}

static float right_wall_angle(void)
{
    float a;

    if (raw_bins[0] == 0 || raw_bins[1] == 0 || raw_bins[0] > 60 || raw_bins[1] > 80)
        return 0.0f;
    a = wall_angle_from(raw_bins[0], raw_bins[1]);
    /* Near a wall end the 30° beam sees past it: huge bogus angle. */
    if (a > 25.0f || a < -25.0f)
        return 0.0f;
    return a;
}

/*
 * Standing still: square up to the right wall (or the left one if the right
 * side is open) so timed turns do not accumulate heading error.
 */
/* Angle to the wall seen at perp_deg / diag_deg; returns 0 if unusable. */
static uint8_t side_angle(uint8_t perp_deg, uint8_t diag_deg, float *a)
{
    uint16_t p, d;

    servo_move(perp_deg);
    p = ping_raw_cm();
    servo_move(diag_deg);
    d = ping_raw_cm();
    if (p == 0 || d == 0 || p >= 45 || d >= 70)
        return 0;
    *a = wall_angle_from(p, d);
    /* A short wall stub or a corner gives a bogus angle. */
    return *a <= ALIGN_MAX_DEG && *a >= -ALIGN_MAX_DEG;
}

static void align_to_wall(void)
{
    float a;

    stop_motors();
    if (side_angle(0, 30, &a)) {
        if (a > 3.0f || a < -3.0f)
            turn_by_deg((int16_t)(-a));
        return;
    }
    if (side_angle(180, 150, &a)) {
        if (a > 3.0f || a < -3.0f)
            turn_by_deg((int16_t)a);
    }
}

/* Timed straight run at DRIVE_PWM; stops early just before hitting a wall. */
static void drive_cm(uint16_t cm)
{
    const float cm_s = ROBOT_MOTOR_RPM_FULL_PWM / 60.0f * 3.14159f
                       * ROBOT_WHEEL_DIAMETER_MM / 10.0f * DRIVE_PWM / 255.0f;
    uint32_t start = hal_millis();
    uint32_t ms = (uint32_t)(cm / cm_s * 1000.0f);

    set_motors(DRIVE_PWM, DRIVE_PWM);
    while (hal_millis() - start < ms) {
        ping_front_only();
        if (filt_f > 0 && filt_f < DRIVE_MIN_FRONT_CM)
            break;
    }
    stop_motors();
}

/*
 * Drive straight with the sonar fixed at 0° until the right wall really ends
 * beside the sensor (30° samples see the gap ~12 cm early).
 * Returns 0 if the wall never ended (false alarm) or the front closed in.
 */
static uint8_t find_right_wall_end(void)
{
    uint8_t i, open_n = 0;
    uint16_t raw;

    servo_move(0);
    set_motors(DRIVE_PWM, DRIVE_PWM);
    for (i = 0; i < WALL_END_MAX_PINGS; i++) {
        raw = ping_raw_cm();
        if (raw == 0 || raw > SIDE_WALL_CM) {
            if (++open_n >= 2)
                break;
        } else {
            open_n = 0;
        }
    }
    stop_motors();
    if (open_n < 2) {
        filt_r = raw;
        return 0;
    }
    filt_r = RANGE_CM;
    return 1;
}

static void apply_fuzzy_drive(void)
{
    FuzzyOut out;
    int16_t base, diff, left, right;

    fuzzy_wall_follow(filt_r, filt_f, FOLLOW_CM, right_wall_angle(), &out);
    base = (int16_t)(DRIVE_PWM * (0.3f + 0.7f * out.speed));
    diff = (int16_t)(DRIVE_PWM * 0.35f * out.steer);
    left = (int16_t)(base - diff);
    right = (int16_t)(base + diff);
    /* Forward only while following. */
    if (left < 0)
        left = 0;
    if (right < 0)
        right = 0;
    if (left > 255)
        left = 255;
    if (right > 255)
        right = 255;
    set_motors(left, right);
}

void robot_setup(void)
{
    uint8_t i;

    mode = MODE_FOLLOW;
    seen_wall = 0;
    turn_loops = 0;
    same_front = 0;
    last_front_raw = 0;
    scan_ord_i = 0;
    miss_l = miss_f = miss_r = 0;
    filt_l = filt_f = filt_r = 0;
    for (i = 0; i < SCAN_BINS; i++)
        raw_bins[i] = 0;

    stop_motors();
    servo_pos = ANG_FRONT;
    hal_servo_write(ANG_FRONT);
    last_ping_ms = 0;
    hal_delay_ms(400);

    /* Prime front and right readings. */
    ping_front_only();
    ping_right_only();
    set_motors(DRIVE_PWM * 2 / 3, DRIVE_PWM * 2 / 3);
    hal_log("RH+fuzzy Striker-style ready");
}

void robot_loop(void)
{
    int wr, wf, wl;

    if (mode == MODE_TURN_LEFT) {
        turn_loops++;
        if (turn_loops >= STUCK_LEFT_TURNS) {
            escape_full_scan();
            return;
        }
        /* Fixed 90°: "front clear" fires early when the echo glances off. */
        turn_by_deg(90);
        ping_front_only();
        ping_front_only();
        if (!wall_front()) {
            align_to_wall();
            ping_right_only();
            ping_front_only();
            stop_motors();
            turn_loops = 0;
            mode = MODE_FOLLOW;
            hal_log("turn L done F=%u", filt_f);
            set_motors(DRIVE_PWM, DRIVE_PWM);
        }
        return;
    }

    if (mode == MODE_TURN_RIGHT) {
        /* Axle past the wall end first, then a clean 90°, then into the gap. */
        align_to_wall();
        if (!find_right_wall_end()) {
            mode = MODE_FOLLOW;
            hal_log("right open false alarm R=%u", filt_r);
            return;
        }
        /* Right opening beats a front wall (L-corner): turn right anyway. */
        drive_cm(RIGHT_PRE_CM);
        turn_by_deg(-90);
        align_to_wall();
        ping_front_only();
        ping_front_only();
        if (wall_front()) {
            mode = MODE_TURN_LEFT;
            return;
        }
        drive_cm(RIGHT_POST_CM);
        ping_right_only();
        ping_right_only();
        turn_loops = 0;
        mode = MODE_FOLLOW;
        hal_log("turn R done R=%u F=%u", filt_r, filt_f);
        if (wall_front()) {
            mode = MODE_TURN_LEFT;
            return;
        }
        set_motors(DRIVE_PWM, DRIVE_PWM);
        return;
    }

    /* MODE_FOLLOW: drive and scan. */
    scan_while_driving();
    if (same_front >= FOLLOW_STUCK_SAMPLES) {
        same_front = 0;
        escape_full_scan();
        return;
    }
    wr = wall_right();
    wf = wall_front();
    wl = wall_left();
    (void)wl;

    if (wr || wf || wl)
        seen_wall = 1;

    /* Striker followWall(onLeft=false): front wall OR lost right wall. */
    if (wf || !wr) {
        stop_motors();
        turn_loops = 0;
        if (wf) {
            mode = MODE_TURN_LEFT;
            hal_log("front wall F=%u -> L", filt_f);
        } else {
            mode = MODE_TURN_RIGHT;
            hal_log("right open R=%u -> R", filt_r);
        }
        return;
    }

    apply_fuzzy_drive();

    if ((scan_ord_i % 4u) == 0)
        hal_log("L=%u F=%u R=%u A=%d follow", filt_l, filt_f, filt_r,
                (int)right_wall_angle());
}
