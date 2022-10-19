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

#ifndef __AIMC_CLUSTER_H__
#define __AIMC_CLUSTER_H__

#include "arch/arm/system.hh"
#include "debug/AIMCCluster.hh"
#include "dev/arm/aimc_dma_controller.hh"
#include "dev/io_device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/AIMCCluster.hh"

#include <vector>

class ArmSystem;
class BaseCPU;
class AIMCDMAController;

enum ActivationFunctions {
    ReLU,
    Sigmoid,
    Tanh,
    Softmax
};

struct AIMCTile {
    // Constructor.
    AIMCTile() : 
    crossbarHeight(2000),
    crossbarWidth(2000),
    crossbar(new int8_t[crossbarWidth*crossbarHeight]),
    inputMemory(new int8_t[crossbarHeight]),
    outputMemory(new int8_t[crossbarWidth]),
    inputMemoryCounter(0),
    outputMemoryCounter(0),
    vectorization(4)
    {
        for (int i = 0; i < crossbarHeight; i++) {
            inputMemory[i] = 0;
            for (int j = 0; j < crossbarWidth; j++) {
                crossbar[(i * crossbarWidth) + j] = 0;
            }
        }

        for (int i = 0; i < crossbarWidth; i++) {
            outputMemory[i] = 0;
        }
    }

    const int crossbarHeight;   // Height of input memory.
    const int crossbarWidth;    // Width of output memory.
    int8_t * crossbar;          // Parameters crossbar.
    int8_t * inputMemory;       // Input memory (pre-DAC).
    int8_t * outputMemory;      // Output memory (post-ADC).
    int inputMemoryCounter;     // Index into input memory.
    int outputMemoryCounter;    // Index into output memory.
    const int vectorization;    // How many values do we queue/dequeue?
};


struct MLPAcceleratorConfig {
    // Constructor.
    MLPAcceleratorConfig() :
    nLayers(2),
    layerIdx(new int[nLayers]),
    layerAct(new ActivationFunctions[nLayers])
    {
        // MLP + ReLU Layer 1.
        layerIdx[0] = 0; // Use AIMC tile 1.
        layerAct[0] = ReLU;

        // MLP + ReLU Layer 2.
        layerIdx[1] = 1; // Use AIMC tile 2.
        layerAct[1] = ReLU;

        // Additional delay for inference (* ticks per ns).
        inferenceDelay = 200 * 1000;
    }

    int nLayers;            // How many layers is this network?
    int * layerIdx;         // Which AIMC tile do we use for each layer?
    // Which activation function do we use for each layer?
    ActivationFunctions * layerAct;
    Tick inferenceDelay;    // What is the inference delay?
};


class AIMCCluster : public BasicPioDevice {
  private:
    // All CPUs
    std::vector<BaseCPU *> cpus;

    // All AIMC tiles (per thread).
    std::vector<AIMCTile *> tiles;

    // Loosely-coupled accelerator configuration.
    MLPAcceleratorConfig * accConfig;

    // System this AIMC cluster to.
    ArmSystem * system;

  public:
    typedef AIMCClusterParams Params;
    const Params * params() const {
        return dynamic_cast<const Params *>(_params);
    }

    AIMCCluster(const Params * p);
    ~AIMCCluster();
    void init() override;

    // AIMC tile operations.
    void aimcProcess(int tid);
    void aimcQueue(int tid, uint32_t val);
    void aimcQueueSingle(int tid, int8_t val);
    uint32_t aimcDequeue(int tid);
    int8_t aimcDequeueSingle(int tid);
    void aimcTransferLayer(int fromLayer, int toLayer);
    int8_t aimcParamRead(int tid, int x, int y);
    void aimcParamWrite(int tid, int x, int y, int8_t val);

    // Accelerator operations.
    void aimcReLU(int tid);

    // Required by SimObject.
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    // Required by BasicPioDevice.
    AddrRangeList getAddrRanges() const override;

    // Required for accessing DMA controller methods.
    friend class AIMCDMAController;
};

#endif // __AIMC_CLUSTER_H__
