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
 *
 *
 * This model is meant to facilitate the DMA interface to the ALPINE custom SoC
 * device.  The preliminary implementation of this interface is to provide easy
 * configurability and otherwise leave the implementation up to the developer
 * for what kind of accelerator exactly (and its associated performance 
 * behaviour) is modeled.
 *
 * Assuming a base address of ADDR and all registers are 4 bytes, the
 * configuration space is programmed as in the following table.  Note that
 * addresses not mentioned are current reserved as "TODO".
 * 
 * | Address       | Register Name   | Description
 * +---------------+-----------------+----------------------------------------+
 * | ADDR + 0x0000 | Base            | Base address, used for debugging.      |
 * | ADDR + 0x0004 | int_num         | GIC interrupt number.                  |
 * | ADDR + 0x0008 | mode            | TODO: Select between burst, cycle      |
 * |               |                 | stealing, and transparent modes [1].   |
 * | ADDR + 0x000C | spawn           | Initiate DMA, then accelerator         |
 * |               |                 | functions if applicable.               |
 * | ADDR + 0x0010 | input_ptr_size  | Input array size (in bytes).           |
 * | ADDR + 0x0014 | output_ptr_size | Output array size (in bytes).          |
 * | ADDR + 0x1000 | input_ptr       | Address of the base of the input       |
 * |               |                 | array.                                 |
 * | ADDR + 0x2000 | output_ptr      | Address of the base of the output      |
 * |               |                 | array.                                 |
 * +---------------+-----------------+----------------------------------------+
 *
 * The rough control flow of this device is as follows:
 * 1. Set up the DMA controller by setting the configuration registers
 *    including with input/output array pointers and sizes, the DMA mode,
 *    interrupt number, and any other custom configurations added in
 *    implementation.
 * 2. Read to the "spawn" register to activate the DMA and associated
 *    functionality of the underlying accelerator.
 * 3. At this point, the CPU should be asleep or doing other work.
 * 4. When the accelerator is finished with its work, this DMA interface
 *    generates a GIC-based external interrupt to wake up/interrupt the CPU.
 *
 * [1] https://www.geeksforgeeks.org/modes-of-dma-transfer/
 *
 */

#ifndef __AIMC_DMA_CONTROLLER_H__
#define __AIMC_DMA_CONTROLLER_H__

#include "arch/arm/system.hh"
#include "debug/AIMCDMAControl.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/aimc_cluster.hh"
#include "dev/io_device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/AIMCDMAController.hh"

class ArmSystem;
class AIMCCluster;

class AIMCDMAController : public AmbaDmaDevice {
  protected:
    // Pointer to system's AIMC cluster object.
    AIMCCluster * aimc_cluster;

    // System this controller belongs to.
    ArmSystem * system;

    // DMA mode enumeration.
    enum dma_mode {
        burst,
        cycle_stealing,
        transparent
    };

    // DMA interface registers.
    uint32_t base;
    uint32_t int_num;
    dma_mode mode;
    uint32_t spawn;
    uint32_t input_ptr;
    uint32_t input_ptr_size;
    uint32_t output_ptr;
    uint32_t output_ptr_size;

    static const uint32_t base_addr             = 0x0000;
    static const uint32_t int_num_addr          = 0x0004;
    static const uint32_t dma_mode_addr         = 0x0008;
    static const uint32_t spawn_addr            = 0x000C;
    static const uint32_t input_ptr_addr        = 0x1000;
    static const uint32_t input_ptr_size_addr   = 0x0010;
    static const uint32_t output_ptr_addr       = 0x2000;
    static const uint32_t output_ptr_size_addr  = 0x0014;

    // Events handling.
    EventFunctionWrapper * inputReadEvent;
    EventFunctionWrapper * outputWriteEvent;

    // Generate GIC interrupt.
    void generateInterrupt();

    // DMA transfer methods for accelerator input/output.
    void dmaTransferInput();
    void dmaTransferInputDone();
    void dmaTransferOutput();
    void dmaTransferOutputDone();

  public:
    typedef AIMCDMAControllerParams Params;
    const Params * params() const {
        return dynamic_cast<const Params *>(_params);
    }

    AIMCDMAController(const Params * p);
    ~AIMCDMAController();

    // Required by AmbaDmaDevice.
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    AddrRangeList getAddrRanges() const override;

    // Required for accessing cluster methods.
    friend class AIMCCluster;
};

#endif // __AIMC_DMA_CONTROLLER_H__
