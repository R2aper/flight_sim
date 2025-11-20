#ifndef LIB_H
#define LIB_H

#include "display.h"
#ifdef DISPLAY_IMPLEMENTATION
#undef DISPLAY_IMPLEMENTATION
#endif
#ifdef DISPLAY_STRIP_PREFIX
#undef DISPLAY_IMPLEMENTATION
#endif

#include "rocketlib/PID.h"
#include "rocketlib/events.h"
#include "rocketlib/fparser.h"
#include "rocketlib/logger.h"
#include "rocketlib/rocket.h"
#include "rocketlib/utils.h"

#include <math.h>

#endif // LIB_H
