#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Same contract as Arduino setup()/loop(). */
void robot_setup(void);
void robot_loop(void);

#ifdef __cplusplus
}
#endif

#endif
