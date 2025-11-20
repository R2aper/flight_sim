#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>

// N*m^2 * kg^-2
#define G 6.67430 * 1e-11

#define _M_E_ 2.7182818284590452354         /* e */
#define _M_LOG2E_ 1.4426950408889634074     /* log_2 e */
#define _M_LOG10E_ 0.43429448190325182765   /* log_10 e */
#define _M_LN2_ 0.69314718055994530942      /* log_e 2 */
#define _M_LN10_ 2.30258509299404568402     /* log_e 10 */
#define _M_PI_ 3.14159265358979323846       /* pi */
#define _M_PI_2_ 1.57079632679489661923     /* pi/2 */
#define _M_PI_4_ 0.78539816339744830962     /* pi/4 */
#define _M_1_PI_ 0.31830988618379067154     /* 1/pi */
#define _M_2_PI_ 0.63661977236758134308     /* 2/pi */
#define _M_2_SQRTPI_ 1.12837916709551257390 /* 2/sqrt(pi) */
#define _M_SQRT2_ 1.41421356237309504880    /* sqrt(2) */
#define _M_SQRT1_2_ 0.70710678118654752440  /* 1/sqrt(2) */

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

void _sleep_(double msec);

void clrscrn();

bool is_almost_integer(double x, double tolerance);

typedef struct vec3_t {
  double x, y, z;

} vec3_t;

#define VEC3_ZERO (vec3_t){0, 0, 0}

#endif // UTILS_H
