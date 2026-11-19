/* Force-included into every application C file (see CMakeLists.txt, #139).
 *
 * printf is the firmware's debug log. In Release (DEBUG_ENABLED == 0) every
 * printf call becomes dead code: it still compiles and type-checks, but the
 * compiler drops it and its format string never reaches the image. Arguments
 * are not evaluated then, so they must not have side effects.
 *
 * <stdio.h> is included first so its own printf declaration is seen before
 * the macro exists; later #include <stdio.h> lines are no-ops. */
#ifndef DEBUG_PRINTF_H
#define DEBUG_PRINTF_H

#include <stdio.h>

#ifndef DEBUG_ENABLED
#define DEBUG_ENABLED 0
#endif

#if !DEBUG_ENABLED
/* The inner printf is not re-expanded (a macro never expands inside itself). */
#define printf(...) do { if (0) { (void)printf(__VA_ARGS__); } } while (0)
#endif

#endif /* DEBUG_PRINTF_H */
