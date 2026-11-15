/* SPDX-License-Identifier: Apache-2.0 */
/* Force-included into every translation unit of the pinned CANopenLinux build (cc -include).
 * Two changes to the shipped canopend, nothing else:
 *  1. NMT_CONTROL without CO_NMT_STARTUP_TO_OPERATIONAL: the node stays Pre-operational after
 *     boot-up until a master starts it. The other bits are the shipped default.
 *  2. Log lines go to stderr instead of syslog (the dev image runs no syslog daemon, so the
 *     shipped build would drop its NMT and emergency log lines).
 */
#ifndef CANOPENNODE_BUILD_H
#define CANOPENNODE_BUILD_H

#define NMT_CONTROL \
    (CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

#include <stdarg.h>
#include <stdio.h>

static inline void co_build_vlog(int priority, const char *format, va_list ap) {
    fprintf(stderr, "<%d>", priority);
    vfprintf(stderr, format, ap);
    fputc('\n', stderr);
    fflush(stderr);
}
#define vsyslog co_build_vlog

#endif
