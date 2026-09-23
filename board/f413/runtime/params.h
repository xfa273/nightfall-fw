#ifndef F413_RUNTIME_PARAMS_H
#define F413_RUNTIME_PARAMS_H

/* Common application code must never inherit one machine's literal defaults.
 * Profile definitions include their own adjacent params.h directly. */
#ifndef NIGHTFALL_F413_RUNTIME_CONFIG
#error "F413 runtime params require boot-selected machine configuration"
#endif
#include "f413_runtime_aliases.h"

#endif
