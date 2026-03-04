#ifndef PM_ORGAN_CORE_MEMORY_ARENA_H
#define PM_ORGAN_CORE_MEMORY_ARENA_H

#include <stddef.h>

#include "pm_organ/core/types.h"

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct MemoryArena
{
    u8 *base;
    usize size;
    usize used;
} MemoryArena;

bool MemoryArena_Create (MemoryArena *arena, usize size);
void MemoryArena_Destroy (MemoryArena *arena);
void MemoryArena_Reset (MemoryArena *arena);
void *MemoryArena_PushSize (MemoryArena *arena, usize size, usize alignment);
void *MemoryArena_PushZeroSize (MemoryArena *arena, usize size, usize alignment);

#define MEMORY_ARENA_PUSH_ARRAY(arena, count, type)                                                                    \
    ((type *) MemoryArena_PushZeroSize((arena), sizeof(type) * (count), _Alignof(type)))

#define MEMORY_ARENA_PUSH_STRUCT(arena, type) ((type *) MemoryArena_PushZeroSize((arena), sizeof(type), _Alignof(type)))

#if defined(__cplusplus)
}
#endif

#endif // PM_ORGAN_CORE_MEMORY_ARENA_H
