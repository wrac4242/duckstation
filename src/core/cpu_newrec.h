#pragma once
#include "types.h"

namespace CPU::NewRec
{
bool Initialize();
void Reset();
void Shutdown();
void Execute();
void InvalidateBlocksWithPageNumber(u32 index);
}

