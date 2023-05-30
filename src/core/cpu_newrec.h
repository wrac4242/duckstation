// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#pragma once
#include "types.h"

#ifdef WITH_NEWREC

namespace CPU::NewRec
{
bool Initialize();
void Reset();
void Shutdown();
void Execute();
void InvalidateAllBlocks();
void InvalidateBlocksWithPageNumber(u32 index);
}

#endif