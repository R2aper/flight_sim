#include "logger.h"

#include "PID.h"
#define DISPLAY_IMPLEMENTATION
#define DISPLAY_STRIP_PREFIX
#include "rocket.h"
#include <stddef.h>

logger_t logger_init(const char *filename) {
  if (!filename)
    return (logger_t){0};

  logger_t l = {0};
  l.filename = filename;

  // Clear file
  l.file = fopen(l.filename, "w");
  if (!l.file)
    return (logger_t){0};
  fclose(l.file);

  l.file = fopen(l.filename, "a");
  if (!l.file)
    return (logger_t){0};

  setvbuf(l.file, NULL, _IOFBF, LOGGER_BUFFER_SIZE);

  return l;
}

int logger_flush(logger_t *l) {
  if (!l || !l->file)
    return -1;

  return fflush(l->file);
}

int logger_free(logger_t *l) {
  if (!l || !l->file)
    return -1;

  int result = fclose(l->file);
  l->file = NULL;

  return result;
}

int logger_write_rocket(logger_t *l, rocket_t *r) {
  if (!l || !l->file || !r)
    return -1;

  r->d.self = r;

  return fprintln(l->file, "{}", r);
}

int logger_write_pid(logger_t *l, PID *pid) {
  if (!l || !l->file || pid)
    return -1;

  pid->d.self = pid;

  return fprintln(l->file, "{}", pid);
}
