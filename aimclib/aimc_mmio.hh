/* 
 * Copyright EPFL 2022
 * Joshua Klein
 * 
 * This file contains wrappers for interfacing the AIMC core via memory-mapped
 * IO.  This assumes the correct installation of the ALPINE MMIO kernel module.
 * The MMIO is interfaced using read/writes to a file pointer.
 *
 * NOTE: Since we do not yet model parameter read/write, we just intrinsics for
 * now.  Furthermore, the concept of an explicit "process call" is now embedded
 * into the device file read operation (since we assume in-order read result 
 * following write input).
 *
 * IMPORTANT NOTE: It is unrealistic to read/write single bytes for performance
 * reasons, use this header only for debugging.
 *
 */

#ifndef __AIMC_MMIO_H__
#define __AIMC_MMIO_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ALPINE_KM_MMIO "/dev/alpine_km_mmio"

FILE * devFile;

// Open file pointer for the KM dev file.
void
initAccelFilePtr()
{
    devFile = fopen(ALPINE_KM_MMIO, "w+");
    return;
}

// Close file pointer for the KM dev file.
void
closeAccelFilePtr()
{
    if (devFile) {
        fclose(devFile);
    }

    return;
}

// Placeholder.
inline void
aimcProcess()
{
    return;
}

/* CM Core Input Memory Queue, assuming opened/non-NULL fptr.
 * Queueing arguments:
 * -- val = Value to be queued into accelerator input memory.
 */
inline void
aimcQueue(int8_t val)
{
    fwrite(&val, sizeof(int8_t), 1, devFile);

    return;
}

/* CM Core Output Memory Dequeue, assuming opened/non-NULL devFile.
 * Note: AIMC process/progress accelerator command is rolled into accelerator
 * model on first read after numerous writes.
 *
 */
inline int8_t
aimcDequeue()
{
    int8_t res;

    fseek(devFile, 0, SEEK_SET);
    if(fread(&res, sizeof(int8_t), 1, devFile)) {
        return res;
    }

    return 0;
}

/* CM Core Parameter Read
 * Instruction format: |____Opcode___|__rm__|_?|__ra__|__rn__|__rd__|
 * Bits:               |31_________21|20__16|15|14__10|9____5|4____0|
 * Binary layout:      |0100_0001_000|0_1000|_1|000_00|01_001|0_1010|
 * Hex layout:         |__4____1____0|____8_|__|_8____|1____2|____A_|
 * gem5 variables:     |_____________|_Op264|__|_Op364|_Op164|Dest64|
 *
 * Queueing arguments:
 * -- rd = Parameter value.
 * -- rm = Parameter x index.
 * -- rn = Parameter y index.
 */
inline uint64_t
aimcParamRead(uint64_t rm, uint64_t rn, uint64_t ra, int tid = 0)
{
    uint64_t res;

    __asm__ volatile(
        "MOV X9, %[input_j];"
        "MOV X7, %[input_k];"
        ".long 0x4108812A;"
        "MOV %[output], X10;"
        : [output] "=r" (res)
        : [input_j] "r" (rm), [input_k] "r" (rn)
        : "x7", "x9", "x10"
    );

    return res;
}

/* CM Core Parameter Write
 * Instruction format: |____Opcode___|__rm__|_?|__ra__|__rn__|__rd__|
 * Bits:               |31_________21|20__16|15|14__10|9____5|4____0|
 * Binary layout:      |0100_0001_000|0_1000|_0|001_11|01_001|0_1010|
 * Hex layout:         |__4____1____0|____8_|__|_1____|D____2|____A_|
 * gem5 variables:     |_____________|_Op264|__|_Op364|_Op164|Dest64|
 *
 * Queueing arguments:
 * -- rd = Success code.
 * -- rm = Parameter x index.
 * -- ra = Parameter value.
 * -- rn = Parameter y index.
 */
inline uint64_t
aimcParamWrite(uint64_t rm, uint64_t rn, uint64_t ra, int tid = 0)
{
    uint64_t res;

    __asm__ volatile(
        "MOV X8, %[input_i];"
        "MOV X9, %[input_j];"
        "MOV X7, %[input_k];"
        ".long 0x41081D2A;"
        "MOV %[output], X10;"
        : [output] "=r" (res)
        : [input_i] "r" (ra), [input_j] "r" (rm), [input_k] "r" (rn)
        : "x7", "x8", "x9", "x10"
    );

    return res;
}

#endif // __AIMC_MMIO_H__
