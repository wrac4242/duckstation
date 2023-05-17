#pragma once
#include "types.h"

namespace CPU::NewRec
{
bool Initialize();
void Execute();
void InvalidateBlocksWithPageNumber(u32 index);
}

