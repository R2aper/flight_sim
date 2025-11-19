#include "utils.h"

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#else
#include <threads.h>
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void _sleep_(double msec) {
#ifdef _WIN32
  Sleep((DWORD)msec);
#else
  struct timespec duration = {.tv_sec = 0, .tv_nsec = (long)(msec * 1e6)};
  thrd_sleep(&duration, NULL);
#endif
}

void clrscrn() {
#ifdef _WIN32
  clrscr();
#else
  // system("clear");
  printf("\033[H\033[J");
#endif
}

bool is_almost_integer(double x, double tolerance) {
  long long n = round(x + 0.5); // rounding
  double diff = x - (double)n;
  if (diff < 0)
    diff = -diff;

  return diff <= tolerance;
}
