#include "fparser.h"

#include <string.h>

fparser_t fparser_init(const char *filename) {
  if (!filename)
    return (fparser_t){0};

  fparser_t fp = {0};
  fp.filename = filename;

  fp.file = fopen(filename, "r");
  if (!fp.file)
    return (fparser_t){0};

  return fp;
}

int fparser_free(fparser_t *fp) {
  if (!fp || !fp->file)
    return -1;

  return fclose(fp->file);
}

int fparser_parse(fparser_t *fp) {
  if (!fp || !fp->file)
    return -1;

  fp->section_count = 0;
  fparser_section_t *current = NULL;

  char line[MAX_LINE];
  while (fgets(line, sizeof(line), fp->file)) {
    // Remove new line symbols
    line[strcspn(line, "\r\n")] = 0;

    // Skip emtpy lines
    if (strlen(line) == 0)
      continue;

    // Check if line is start of the sections
    if (line[0] == '[') {
      if (fp->section_count >= MAX_SECTIONS) {
        // Stop parsing if max sections are reached
        continue;
      }
      char name[MAX_NAME];
      if (sscanf(line, "[%63[^]]]", name) == 1) {
        current = &fp->sections[fp->section_count++];
        strncpy(current->name, name, MAX_NAME);
        current->var_count = 0;
      }
    } else if (current) { // Otherwise if declaration of variable
      if (current->var_count >= MAX_VARS) {
        // Stop parsing variables for this section if max are reached
        continue;
      }
      char varname[MAX_NAME];
      double value;
      if (sscanf(line, "%63s = %lf", varname, &value) == 2) {
        fparser_var_t *v = &current->vars[current->var_count++];
        strncpy(v->name, varname, MAX_NAME);
        v->value = value;
      }
    }
  }

  return 0;
}

fparser_section_t fparser_get_section(fparser_t *fp, const char *section_name) {
  if (!fp || !fp->file)
    return (fparser_section_t){0};

  for (int i = 0; i < fp->section_count; i++) {
    if (strcmp(fp->sections[i].name, section_name) == 0)
      return fp->sections[i];
  }

  return (fparser_section_t){0};
}

fparser_var_t fparser_get_var(fparser_t *fp, const char *section_name,
                              const char *var_name) {
  if (!fp || !fp->file)
    return (fparser_var_t){0};

  for (int i = 0; i < fp->section_count; i++) {
    if (strcmp(fp->sections[i].name, section_name) == 0) {
      for (int j = 0; j < fp->sections[i].var_count; j++) {
        if (strcmp(fp->sections[i].vars[j].name, var_name) == 0) {
          return fp->sections[i].vars[j];
        }
      }
    }
  }

  return (fparser_var_t){0};
}
