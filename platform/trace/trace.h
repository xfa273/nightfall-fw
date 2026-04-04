#ifndef NIGHTFALL_TRACE_H_
#define NIGHTFALL_TRACE_H_

#include <stddef.h>

void trace_init(void);
void trace_write(const char* data, size_t len);
int trace_printf(const char* fmt, ...);

#endif
