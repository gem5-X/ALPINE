/*
 * Copyright (c) 2022 EPFL
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

#include "dev/arm/aimc_dma_controller.hh"


// Generate GIC interrupt.
void
AIMCDMAController::generateInterrupt()
{
    gic->sendInt(int_num);

    return;
}

// DMA transfer methods for accelerator input.
// TODO: Implement different execution based on DMA mode.
void
AIMCDMAController::dmaTransferInput()
{
    if (input_ptr && input_ptr_size) {
        // Prototype: dmaRead(Addr addr, int size, Event *event,
        //              uint8_t *data, Tick delay = 0)
        inputReadEvent = new EventFunctionWrapper(
            [this]{ dmaTransferInputDone(); }, name());
        dmaRead(input_ptr,
            input_ptr_size,
            inputReadEvent,
            (uint8_t *)aimc_cluster->tiles[0]->inputMemory,
            1000);
    } else {
        warn("AIMC DMA controller input parameters invalid!");
    }

    return;
}

// Method is called when input array transfer is complete.
void
AIMCDMAController::dmaTransferInputDone()
{
    generateInterrupt();

    return;
}

// DMA transfer methods for accelerator output.
// TODO: Implement different execution based on DMA mode.
void
AIMCDMAController::dmaTransferOutput()
{
    if (output_ptr && output_ptr_size) {
        // Prototype: dmaWrite(Addr addr, int size, Event *event,
        //              uint8_t *data, Tick delay = 0)
        outputWriteEvent = new EventFunctionWrapper(
            [this]{ dmaTransferOutputDone(); }, name());
        dmaWrite(output_ptr,
            output_ptr_size,
            outputWriteEvent,
            (uint8_t *)aimc_cluster->tiles[0]->outputMemory,
            1000);
    } else {
        warn("AIMC DMA controller output parameters invalid!");
    }

    return;
}

// Method is called when output array transfer is complete.
void
AIMCDMAController::dmaTransferOutputDone()
{
    generateInterrupt();

    return;
}

// Constructor.
AIMCDMAController::AIMCDMAController(
	const AIMCDMAControllerParams * p) :
    AmbaDmaDevice(p),
    aimc_cluster(p->aimc_cluster),
	system(dynamic_cast<ArmSystem *>(p->system)) 
{
	warn("AIMC DMA interface instantiated.");
    if (aimc_cluster) warn("AIMC cluster linked.");

    this->pioAddr = p->pio_addr;
}

// Destructor.
AIMCDMAController::~AIMCDMAController()
{
    return;
}

// Read to AIMC cluster based on packet interation.
Tick
AIMCDMAController::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr addr = pkt->getAddr() - pioAddr;

    warn("AIMC cluster DMA controller read at address %#x.", addr);

    switch (addr) {
        // base
        case base_addr:
        pkt->set<uint32_t>(base);
        break;

        case int_num_addr:
        pkt->set<uint32_t>(int_num);
        break;

        case dma_mode_addr:
        pkt->set<uint32_t>((uint32_t)mode);
        break;

        case spawn_addr:
        // Activate accelerator functionality.
        dmaTransferInput();

        // Send back "success".
        pkt->set<uint32_t>(1);
        break;

        case input_ptr_addr:
        pkt->set<uint32_t>(input_ptr);
        break;

        case input_ptr_size_addr:
        pkt->set<uint32_t>(input_ptr_size);
        break;

        case output_ptr_addr:
        pkt->set<uint32_t>(output_ptr);
        break;

        case output_ptr_size_addr:
        pkt->set<uint32_t>(output_ptr_size);
        break;

        default:
        warn("AIMCDMAController::read to unknown address!");
        break;
    }

    pkt->makeAtomicResponse();
	return pioDelay;
}

// Write to AIMC cluster based on packet interation.
Tick
AIMCDMAController::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr addr = pkt->getAddr() - pioAddr;
    uint32_t data = pkt->get<uint32_t>();

    warn("AIMC cluster DMA controller write %d at address %#x.", (int)data, addr);

    switch (addr) {
        case base_addr:
        warn("No functionality for writing the base address!");
        break;

        case int_num_addr:
        int_num = data;
        warn("Note: Unable to change GIC interrupt number in Params object for"
            "AIMC cluster DMA controller.");
        break;

        case dma_mode_addr:
        if (mode < 3) {
            mode = (dma_mode)data;
        } else {
            warn("Tried to write invalid mode to DMA wrapper mode register!");
        }
        break;

        case spawn_addr:
        dmaTransferInput();
        break;

        case input_ptr_addr:
        input_ptr = data;
        break;

        case input_ptr_size_addr:
        input_ptr_size = data;
        break;

        case output_ptr_addr:
        output_ptr = data;
        break;

        case output_ptr_size_addr:
        output_ptr_size = data;
        break;

        default:
        warn("AIMCDMAController::write to unknown address!");
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
AIMCDMAController::serialize(CheckpointOut &cp) const
{
    warn("AIMCDMAController::serialize not yet implemented.");

    return;
}

void
AIMCDMAController::unserialize(CheckpointIn &cp)
{
    warn("AIMCDMAController::unserialize not yet fully implemented.");

    // Re-link system/AIMC cluster pointers.
    const Params * p = this->params();
    system = dynamic_cast<ArmSystem *>(p->system);
    aimc_cluster = system->getAIMCCluster();

    return;
}

// Returns address range of AIMC cluster, required for the bridge to be able to
// find the AIMC cluster device.
AddrRangeList
AIMCDMAController::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;

}

AIMCDMAController *
AIMCDMAControllerParams::create()
{
	return new AIMCDMAController(this);
}