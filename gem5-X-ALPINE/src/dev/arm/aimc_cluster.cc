/*
 * Copyright (c) 2020 EPFL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Joshua Klein
 *          Marina Zapater
 *          David Atienza
 */

#include "dev/arm/aimc_cluster.hh"

// Constructor.
AIMCCluster::AIMCCluster(
	const AIMCClusterParams * p) :
	BasicPioDevice(p,  p->pio_size),
	system(dynamic_cast<ArmSystem *>(p->system))
{
	warn("AIMC tile instantiated.");

    this->pioAddr = p->pio_addr;
    this->pioSize = p->pio_size;

	for (auto cpu : p->cpus) {
        cpus.push_back(cpu);
        tiles.push_back(new AIMCTile());
    }
}

// Destroy AIMC tiles and associated objects.
AIMCCluster::~AIMCCluster()
{
    for (auto tile : tiles) {
    	delete[] tile->inputMemory;
    	delete[] tile->outputMemory;
    	delete[] tile->crossbar;
        delete[] tile;
    }
}

// AIMC cluster initialization.
void
AIMCCluster::init()
{
	BasicPioDevice::init();
	system->setAIMCCluster(this);
}

// Perform M x V using TID's AIMC tile crossbar and input memory, store result
// in output memory.
// NOTE: Also refreshes input memory for future AIMC process use.
void
AIMCCluster::aimcProcess(int tid)
{
	int height = tiles[tid]->crossbarHeight;
    int width = tiles[tid]->crossbarWidth;

    for (int i = 0; i < width; i++) {
        double acc = 0.0;

        for (int j = 0; j < height; j++) {
            acc += (tiles[tid]->inputMemory[j]) * tiles[tid]->crossbar[(j * width) + i];
        }

        if (acc > 127) tiles[tid]->outputMemory[i] = 127;
        else if (acc < -128) tiles[tid]->outputMemory[i] = -128;
        else tiles[tid]->outputMemory[i] = (int8_t)acc;
    }

    // Refresh input memory and queue counters.
    for (int i = 0; i < height; i++) {
        tiles[tid]->inputMemory[i] = 0;
    }

    tiles[tid]->inputMemoryCounter = 0;
    tiles[tid]->outputMemoryCounter = 0;

    return;
}

// Queue 8-bit digital value into TID's AIMC tile input memory.
void
AIMCCluster::aimcQueue(int tid, uint32_t val)
{
    int idx = tiles[tid]->inputMemoryCounter;
    int length = tiles[tid]->vectorization;

	if ((idx + length) < tiles[tid]->crossbarHeight) {
        for (int i = 0; i < length; i++) {
            int8_t currVal = (val >> (8 * i)) & 0xff;
            tiles[tid]->inputMemory[idx + i] = currVal;
        }
    }

    tiles[tid]->inputMemoryCounter += length;

    return;
}

// Queue single 8-bit digital value into TID's AIMC tile input memory.
void
AIMCCluster::aimcQueueSingle(int tid, int8_t val)
{
    int idx = tiles[tid]->inputMemoryCounter;

    if ((idx) < tiles[tid]->crossbarHeight) {
        tiles[tid]->inputMemory[idx] = val;
    }

    tiles[tid]->inputMemoryCounter++;

    return;
}

// Dequeue 8-bit digital value out of TID's AIMC tile output memory.
uint32_t
AIMCCluster::aimcDequeue(int tid)
{
    uint32_t result = 0;
    int idx = tiles[tid]->outputMemoryCounter;
    int length = tiles[tid]->vectorization;

    if ((idx + length) < tiles[tid]->crossbarWidth) {
        for (int i = length-1; i > -1; i--) {
            result <<= 8;
            result |= 0xff & (tiles[tid]->outputMemory[idx + i]);
        }
    }

    tiles[tid]->outputMemoryCounter += length;

    return result;
}

// Dequeue single 8-bit digital value out of TID's AIMC tile output memory.
int8_t
AIMCCluster::aimcDequeueSingle(int tid)
{
    int8_t result = 0;
    int idx = tiles[tid]->outputMemoryCounter;

    if ((idx + 1) < tiles[tid]->crossbarWidth) {
        result = tiles[tid]->outputMemory[idx];
    }

    tiles[tid]->outputMemoryCounter++;

    return result;
}

// Read parameter at TID's AIMC tile's crossbar[x, y].
int8_t
AIMCCluster::aimcParamRead(int tid, int x, int y)
{
    int8_t result = 0;
    int height = tiles[tid]->crossbarHeight;
    int width = tiles[tid]->crossbarWidth;
    int idx = (y * width) + x; // 1D index

    if (idx < width * height) {
        result = tiles[tid]->crossbar[idx];
    }

    return result;
}

// Write parameter val at TID's AIMC tile's crossbar[x, y].
void
AIMCCluster::aimcParamWrite(int tid, int x, int y, int8_t val)
{
	int height = tiles[tid]->crossbarHeight;
    int width = tiles[tid]->crossbarWidth;
    int idx = (y * width) + x; // 1D index

    if (idx < width * height) {
        tiles[tid]->crossbar[idx] = val;
    }

    return;
}

// Read to AIMC cluster based on packet interation.
Tick
AIMCCluster::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr addr = pkt->getAddr() - pioAddr;

    warn("AIMC cluster read at address %#x.", addr);

    pkt->makeAtomicResponse();
	return pioDelay;
}

// Write to AIMC cluster based on packet interation.
Tick
AIMCCluster::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    Addr addr = pkt->getAddr() - pioAddr;
    uint8_t data = pkt->get<uint8_t>();

    warn("AIMC cluster write %d at address %#x.", (int)data, addr);

    pkt->makeAtomicResponse();
	return pioDelay;
}

// Serialize AIMC cluster.
void
AIMCCluster::serialize(CheckpointOut &cp) const
{
	warn("Full AIMC cluster config serialization not yet implemented; "
        "destroying old instances.");

    // TODO: Actually serialize AIMC tiles and contents.  For now, the AIMC tiles
    // have to refreshed after checkpoint.  Same with accelerator configs.
    for (auto tile : tiles) {
        delete[] tile->inputMemory;
        delete[] tile->outputMemory;
        delete[] tile->crossbar;
        delete[] tile;
    }
}

// Unserialize AIMC cluster.
void
AIMCCluster::unserialize(CheckpointIn &cp)
{
	warn("AIMC cluster accelerator config deserialization not yet implemented; "
        "creating new instances.");

    // TODO: Actually unserialize AIMC tiles and contents/accelerator config.
    // Init AIMC tiles.
    const Params * p = this->params();

    if (p->system) warn("AIMC cluster system param added.");

    system = dynamic_cast<ArmSystem *>(p->system);
    this->init();
    for (auto cpu : p->cpus) {
        if (cpu) warn("Adding CPU AIMC tile.");
        cpus.push_back(cpu);
        tiles.push_back(new AIMCTile());
    }
}

// Returns address range of AIMC cluster, required for the bridge to be able to
// find the AIMC cluster device.
AddrRangeList
AIMCCluster::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

AIMCCluster *
AIMCClusterParams::create()
{
	return new AIMCCluster(this);
}