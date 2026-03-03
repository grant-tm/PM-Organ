#ifndef PM_ORGAN_CORE_ASSERT_H
#define PM_ORGAN_CORE_ASSERT_H

#include <stdlib.h>

#if defined(_MSC_VER)
    #define DEBUG_BREAK() __debugbreak()
#else
    #define DEBUG_BREAK() abort()
#endif

#define ASSERT(expression)                                                                                               \
    do                                                                                                                   \
    {                                                                                                                    \
        if (!(expression))                                                                                               \
        {                                                                                                                \
            DEBUG_BREAK();                                                                                               \
            abort();                                                                                                     \
        }                                                                                                                \
    } while (0)

#endif // PM_ORGAN_CORE_ASSERT_H
