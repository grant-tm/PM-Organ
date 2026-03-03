#ifndef PM_ORGAN_CORE_MEMORY_ARENA_H
#define PM_ORGAN_CORE_MEMORY_ARENA_H

#include <stddef.h>

#include "pm_organ/core/types.h"

typedef struct MemoryArena
{
    u8 *base;
    size_t size;
    size_t used;
} MemoryArena;

bool MemoryArena_Create (MemoryArena *arena, size_t size);
void MemoryArena_Destroy (MemoryArena *arena);
void MemoryArena_Reset (MemoryArena *arena);
void *MemoryArena_PushSize (MemoryArena *arena, size_t size, size_t alignment);
void *MemoryArena_PushZeroSize (MemoryArena *arena, size_t size, size_t alignment);

#define MEMORY_ARENA_PUSH_STRUCT(arena, type) ((type *) MemoryArena_PushZeroSize((arena), sizeof(type), _Alignof(type)))

#endif // PM_ORGAN_CORE_MEMORY_ARENA_H
