#ifndef FPARSER_H
#define FPARSER_H

#include "utils.h"

#include "stdio.h"

/*
 * @file fparser.h
 * @brief Simple file parser for getting info for simulations
 *
 * This file contains structures and functions for parsing files of the following format:
 * [struct1]
 * var1 = 1.0
 * var2 = 65.3
 *
 * [struct2]
 * var3 = 5.1
 * ...
 */

#define MAX_LINE 256
#define MAX_NAME 64

#define MAX_VARS 64
#define MAX_SECTIONS 64

/**
 * @struct fparser_var_t
 * @brief Represents a variable (key-value pair) within a section
 *
 */
typedef struct fparser_var_t {
  char name[MAX_NAME];
  double value;

} fparser_var_t;

/**
 * @struct fparser_section_t
 * @brief Represents a section containing multiple variables
 *
 */
typedef struct fparser_section_t {
  char name[MAX_NAME];
  fparser_var_t vars[MAX_VARS];
  int var_count;

} fparser_section_t;

/**
 * @struct fparser_t
 * @brief Represents the parser state, including all sections read from a file
 *
 */
typedef struct fparser_t {
  FILE *file;
  fparser_section_t sections[MAX_SECTIONS];
  const char *filename;
  int section_count;

} fparser_t;

fparser_t fparser_init(const char *filename);
int fparser_free(fparser_t *fp);

int fparser_parse(fparser_t *fp);

fparser_section_t fparser_get_section(fparser_t *fp, const char *section_name);
fparser_var_t fparser_get_var(fparser_t *fp, const char *section_name, const char *var_name);

#endif // FPARSER_H