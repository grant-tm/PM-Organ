#include <stdlib.h>
#include <string.h>

#include "pm_organ/core/assert.h"
#include "pm_organ/core/memory_arena.h"

static size_t AlignForward (size_t value, size_t alignment)
{
    ASSERT(alignment > 0);
    ASSERT((alignment & (alignment - 1)) == 0);

    return (value + (alignment - 1)) & ~(alignment - 1);
}

bool MemoryArena_Create (MemoryArena *arena, size_t size)
{
    ASSERT(arena != NULL);
    ASSERT(size > 0);

    arena->base = (u8 *) malloc(size);
    if (arena->base == NULL)
    {
        arena->size = 0;
        arena->used = 0;
        return false;
    }

    arena->size = size;
    arena->used = 0;
    return true;
}

void MemoryArena_Destroy (MemoryArena *arena)
{
    ASSERT(arena != NULL);

    free(arena->base);

    arena->base = NULL;
    arena->size = 0;
    arena->used = 0;
}

void MemoryArena_Reset (MemoryArena *arena)
{
    ASSERT(arena != NULL);

    arena->used = 0;
}

void *MemoryArena_PushSize (MemoryArena *arena, size_t size, size_t alignment)
{
    size_t aligned_used;
    size_t next_used;

    ASSERT(arena != NULL);
    ASSERT(arena->base != NULL);
    ASSERT(size > 0);

    aligned_used = AlignForward(arena->used, alignment);
    next_used = aligned_used + size;

    if (next_used > arena->size)
    {
        ASSERT(false);
        return NULL;
    }

    arena->used = next_used;
    return arena->base + aligned_used;
}

void *MemoryArena_PushZeroSize (MemoryArena *arena, size_t size, size_t alignment)
{
    void *memory;

    memory = MemoryArena_PushSize(arena, size, alignment);
    if (memory == NULL)
    {
        return NULL;
    }

    memset(memory, 0, size);
    return memory;
}
