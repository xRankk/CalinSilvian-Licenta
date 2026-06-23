#ifndef MOTORS_H
#define MOTORS_H
#include "Config.h"

void initMotors();
void pas();
void forezaStop();
void forezaStartFwd();
void forezaStartRev();
void forezaRampUpdate();
void controlFans(int v);

#endif
