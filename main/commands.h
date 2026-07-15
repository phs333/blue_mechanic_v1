#pragma once

#include "app_defs.h"

void commands_print_help(void);
void commands_print_status(app_context_t *ctx);
void commands_handle_line(app_context_t *ctx, const char *line);
