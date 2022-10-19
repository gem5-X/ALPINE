/* 
 * Copyright EPFL 2021
 * Joshua Klein
 * 
 * This file contains all of the includes for which to build the AIMC library.
 *
 */

#ifndef __AIMC_HH__
#define __AIMC_HH__

#if defined (USE_CHECKER) // Are we simulating the gem5 code w/ ISA extension?
#include "aimc_check.hh"
#else // (USE_CHECKER)

#if defined (LOOSELY_COUPLED_MMIO)
#include "aimc_mmio.hh"
#else // (LOOSELY_COUPLED_MMIO)
#include "aimc_intrinsics.hh"
#endif // (LOOSELY_COUPLED_MMIO)

#endif // (USE_CHECKER)

#include "aimc_queue.hh"
#include "aimc_dequeue.hh"
#include "aimc_tile.hh"

#if defined (LOOSELY_COUPLED_MMIO)
#else // (LOOSELY_COUPLED_MMIO)
#include "aimc_conv.hh"
#endif // (LOOSELY_COUPLED_MMIO)

#endif // __AIMC_HH__
