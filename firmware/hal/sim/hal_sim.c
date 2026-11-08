/*
 * HAL for the simulator. The simulator starts this program as a child
 * process and is the only clock: every call is a request on stdout and the
 * program blocks until the reply arrives on stdin. Protocol (one line each):
 *
 *   robot -> sim          sim -> robot
 *   HELLO 1               OK
 *   MS                    T <millis>
 *   US                    T <micros>
 *   DLY <ms>              OK            (after <ms> of simulated time)
 *   DLYU <us>             OK
 *   MOT <left> <right>    OK
 *   SRV <deg>             OK
 *   PING                  PONG <echo_us>
 *   LOG <text>            (no reply)
 *
 * The real stdout is moved to a private descriptor in hal_init(), so printf()
 * in robot code goes to stderr and shows up in the simulator log.
 */
#include "hal.h"
#include "robot.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static FILE *sim_out;
static char reply[128];

static void tx(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(sim_out, fmt, ap);
    va_end(ap);
    fputc('\n', sim_out);
    fflush(sim_out);
}

static const char *rx(const char *prefix)
{
    size_t n = strlen(prefix);

    if (!fgets(reply, sizeof reply, stdin))
        exit(0); /* simulator closed the pipe: reset or quit */
    reply[strcspn(reply, "\r\n")] = '\0';
    if (strncmp(reply, prefix, n) != 0) {
        fprintf(stderr, "[hal_sim] expected '%s', got '%s'\n", prefix, reply);
        exit(2);
    }
    return reply + n;
}

void hal_init(void)
{
    int fd = dup(STDOUT_FILENO);

    sim_out = fdopen(fd, "w");
    if (!sim_out) {
        perror("fdopen");
        exit(2);
    }
    dup2(STDERR_FILENO, STDOUT_FILENO);
    setvbuf(stdout, NULL, _IONBF, 0);
    tx("HELLO 1");
    rx("OK");
}

uint32_t hal_millis(void)
{
    tx("MS");
    return (uint32_t)strtoul(rx("T "), NULL, 10);
}

uint32_t hal_micros(void)
{
    tx("US");
    return (uint32_t)strtoul(rx("T "), NULL, 10);
}

void hal_delay_ms(uint32_t ms)
{
    tx("DLY %lu", (unsigned long)ms);
    rx("OK");
}

void hal_delay_us(uint32_t us)
{
    tx("DLYU %lu", (unsigned long)us);
    rx("OK");
}

void hal_motors(int16_t left, int16_t right)
{
    tx("MOT %d %d", left, right);
    rx("OK");
}

void hal_servo_write(uint8_t angle_deg)
{
    tx("SRV %u", angle_deg);
    rx("OK");
}

uint32_t hal_sonar_echo_us(void)
{
    tx("PING");
    return (uint32_t)strtoul(rx("PONG "), NULL, 10);
}

void hal_log(const char *fmt, ...)
{
    char buf[120];
    char *p;
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    for (p = buf; *p; p++)
        if (*p == '\n' || *p == '\r')
            *p = ' ';
    tx("LOG %s", buf);
}

int main(void)
{
    hal_init();
    robot_setup();
    for (;;)
        robot_loop();
}
