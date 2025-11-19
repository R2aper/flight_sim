#ifndef LOGGER_H
#define LOGGER_H

/*
 * @file logger.h
 * @brief A simple buffered file logger
 *
 * This file provides functionality for logging data to a file with an
 * in-memory buffer to reduce the number of direct file I/O operations
 * The logger is designed to be flushed periodically or when the buffer is full
 */

#include <stdio.h>

typedef struct rocket_t rocket_t;
typedef struct PID PID;

// Default buffer: 64 KB
#define LOGGER_BUFFER_SIZE (64 * 1024)

/**
 * @struct logger_t
 * @brief Represents a file logger with an internal buffer
 *
 */
typedef struct logger_t {
  FILE *file;
  const char *filename;

} logger_t;

logger_t logger_init(const char *filename);
int logger_free(logger_t *l);
/// @brief Flushes the write buffer to the file.
int logger_flush(logger_t *l);

int logger_write_rocket(logger_t *l, rocket_t *r);
int logger_write_pid(logger_t *l, PID *pid);

#endif // LOGGER_H