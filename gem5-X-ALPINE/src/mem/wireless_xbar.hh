/*
 * Copyright (c) 2021 EPFL
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
 * Authors: Rafael Medina
 */

/**
 * @file
 * Declaration of a wireless coherent crossbar.
 */

#ifndef __MEM_WIRELESS_XBAR_HH__
#define __MEM_WIRELESS_XBAR_HH__

#include <deque>
#include <unordered_map>
#include <unordered_set>

#include "enums/MacProtocol.hh"
#include "enums/LayerType.hh"
#include "mem/packet.hh"
#include "mem/qport.hh"
#include "mem/snoop_filter.hh"
#include "mem/xbar.hh"
#include "params/WirelessXBar.hh"
#include "sim/probe/mem.hh"
#include "sim/stats.hh"

class WirelessXBar : public BaseXBar
{

  protected:

    template <typename SrcType, typename DstType>
    class WirelessLayer : public Drainable
    {
      public:

        /**
         * Create a wireless layer and give it a name. The layer uses
         * the crossbar an event manager.
         *
         * @param _port destination port the layer converges at
         * @param _xbar the crossbar this layer belongs to
         * @param _name the layer's name
         */
        WirelessLayer(DstType& _port, WirelessXBar& _xbar, 
                      const std::string& _name, Enums::MacProtocol _macProto);

        /**
         * Drain according to the normal semantics, so that the crossbar
         * can tell the layer to drain, and pass an event to signal
         * back when drained.
         *
         * @param de drain event to call once drained
         *
         * @return 1 if busy or waiting to retry, or 0 if idle
         */
        DrainState drain() override;

        /**
         * Get the crossbar layer's name
         */
        const std::string name() const { return xbar.name() + _name; }

        /**
         * Determine if the layer accepts a packet from a specific
         * port. If not, the port in question is also added to the
         * retry list. The acceptance depends on the MAC protocol employed.
         *
         * @param port Source port presenting the packet
         *
         * @return True if the layer accepts the packet
         */
        bool tryTiming(SrcType* src_port, PortID src_id, PortID dest_port,
                      PacketPtr pkt, Enums::LayerType layerType);

        // @TODO we will probably need a tryTiming wrapper for wired layers,
        // as the packet pointer is not included in their calls

        /**
         * Deal with a destination port accepting a packet by potentially
         * removing the source port from the retry list (if retrying).
         *
         * @param busy_time Time to spend as a result of a successful send
         */
        void succeededTiming(Tick busy_time);

        /**
         * Deal with a destination port not accepting a packet by
         * potentially adding the source port to the retry list (if
         * not already at the front).
         *
         * @param src_port Source port
         * @param busy_time Time to spend as a result of a failed send
         */
        void failedTiming(SrcType* src_port, Tick busy_time);

        /** Occupy the layer until until */
        void occupyLayer(Tick until);

        /**
         * @TODO: what this will do?
         */
        void retryWaiting();

        /**
         * Handle a retry from a neighbouring module.
         */
        void recvRetry();

        /**
         * After resolving a collision through exponential backoff, trigger
         * the sendRetry through the correct src_port
         */
        void triggerSendRetry(Port* srcPort);

        /**
         * Register stats for the layer
         */
        void regStats();

      protected:

        /**
         * Sending the actual retry, in a manner specific to the
         * individual layers. Note that for a MasterPort, there is
         * both a RequestLayer and a SnoopResponseLayer using the same
         * port, but using different functions for the flow control.
         */
        virtual void sendRetry(SrcType* retry_port) = 0;

      private:

        /** The destination port this layer converges at. */
        DstType& port;

        /** The crossbar this layer is a part of. */
        WirelessXBar& xbar;

        /** A name for this layer. */
        std::string _name;

        /** Stores which MAC protocol is being modelled */
        const Enums::MacProtocol macProto;

        /**
         * We declare an enum to track the state of the layer. The
         * starting point is an idle state where the layer is waiting
         * for a packet to arrive and is has no scheduled event. Upon 
         * arrival, the layer transitions to the busy state, where it
         * remains until no transmission is being handled by the layer.
         */
        enum State { IDLE, BUSY };

        /** track the state of the layer */
        State state;

        /**
         * A deque of ports and pkts that retry should be called on because
         * the original send was delayed due to a busy layer.
         */
        std::deque<Port*> waitingForLayer;

        /**
         * Track who is waiting for the retry when receiving it from a
         * peer. DONE more than one peers can have denied transmission
         */
        std::deque<SrcType*> waitingForPeer;

        /**
         * Release the layer after being occupied and return to an
         * idle state where we proceed to send a retry to any
         * potential waiting port, or drain if asked to do so.
         */
        void releaseLayer();

        /** event used to schedule a release of the layer */
        EventFunctionWrapper releaseEvent;

        /**
         * Stats for occupancy and utilization. These stats capture
         * the time the layer spends in the busy state and are thus only
         * relevant when the memory system is in timing mode.
         */
        Stats::Scalar occupancy;
        Stats::Formula utilization;
    };

    class ReqWirelessLayer : public WirelessLayer<SlavePort,MasterPort>
    {
      public:
        /**
         * Create a request layer and give it a name.
         *
         * @param _port destination port the layer converges at
         * @param _xbar the crossbar this layer belongs to
         * @param _name the layer's name
         */
        ReqWirelessLayer(MasterPort& _port, WirelessXBar& _xbar,
                      const std::string& _name, Enums::MacProtocol _macProto) :
            WirelessLayer(_port, _xbar, _name, _macProto) {}

      protected:

        void sendRetry(SlavePort* retry_port)
        { retry_port->sendRetryReq(); }
    };

    class RespWirelessLayer : public WirelessLayer<MasterPort,SlavePort>
    {
      public:
        /**
         * Create a response layer and give it a name.
         *
         * @param _port destination port the layer converges at
         * @param _xbar the crossbar this layer belongs to
         * @param _name the layer's name
         */
        RespWirelessLayer(SlavePort& _port, WirelessXBar& _xbar,
                      const std::string& _name, Enums::MacProtocol _macProto) :
            WirelessLayer(_port, _xbar, _name, _macProto) {}

      protected:

        void sendRetry(MasterPort* retry_port)
        { retry_port->sendRetryResp(); }
    };

    class SnoopRespWirelessLayer : public WirelessLayer<SlavePort,MasterPort>
    {
      public:
        /**
         * Create a snoop response layer and give it a name.
         *
         * @param _port destination port the layer converges at
         * @param _xbar the crossbar this layer belongs to
         * @param _name the layer's name
         */
        SnoopRespWirelessLayer(MasterPort& _port, WirelessXBar& _xbar,
                      const std::string& _name, Enums::MacProtocol _macProto) :
            WirelessLayer(_port, _xbar, _name, _macProto) {}

      protected:

        void sendRetry(SlavePort* retry_port)
        { retry_port->sendRetrySnoopResp(); }
    };

    /**
     * Declare the layers of this crossbar, one vector for requests,
     * one for responses, and one for snoop responses @TODO: to modify
     * for parametrization afterwards
     */
    std::vector<ReqWirelessLayer*> reqLayers;
    std::vector<RespWirelessLayer*> respLayers;
    std::vector<SnoopRespWirelessLayer*> snoopLayers;

    /**
     * Declaration of the coherent crossbar slave port type, one will
     * be instantiated for each of the master ports connecting to the
     * crossbar.
     */
    class WirelessXBarSlavePort : public QueuedSlavePort
    {

      private:

        /** A reference to the crossbar to which this port belongs. */
        WirelessXBar &xbar;

        /** A normal packet queue used to store responses. */
        RespPacketQueue queue;

      public:

        WirelessXBarSlavePort(const std::string &_name,
                             WirelessXBar &_xbar, PortID _id)
            : QueuedSlavePort(_name, &_xbar, queue, _id), xbar(_xbar),
              queue(_xbar, *this)
        { }

      protected:

        /**
         * When receiving a timing request, pass it to the crossbar.
         */
        virtual bool recvTimingReq(PacketPtr pkt)
        { return xbar.recvTimingReq(pkt, id); }

        /**
         * When receiving a timing snoop response, pass it to the crossbar.
         */
        virtual bool recvTimingSnoopResp(PacketPtr pkt)
        { return xbar.recvTimingSnoopResp(pkt, id); }

        /**
         * When receiving an atomic request, pass it to the crossbar.
         */
        virtual Tick recvAtomic(PacketPtr pkt)
        { return xbar.recvAtomic(pkt, id); }

        /**
         * When receiving a functional request, pass it to the crossbar.
         */
        virtual void recvFunctional(PacketPtr pkt)
        { xbar.recvFunctional(pkt, id); }

        /**
         * Return the union of all adress ranges seen by this crossbar.
         */
        virtual AddrRangeList getAddrRanges() const
        { return xbar.getAddrRanges(); }

    };

    /**
     * Declaration of the coherent crossbar master port type, one will be
     * instantiated for each of the slave interfaces connecting to the
     * crossbar.
     */
    class WirelessXBarMasterPort : public MasterPort
    {
      private:
        /** A reference to the crossbar to which this port belongs. */
        WirelessXBar &xbar;

      public:

        WirelessXBarMasterPort(const std::string &_name,
                              WirelessXBar &_xbar, PortID _id)
            : MasterPort(_name, &_xbar, _id), xbar(_xbar)
        { }

      protected:

        /**
         * Determine if this port should be considered a snooper. For
         * a coherent crossbar master port this is always true.
         *
         * @return a boolean that is true if this port is snooping
         */
        virtual bool isSnooping() const
        { return true; }

        /**
         * When receiving a timing response, pass it to the crossbar.
         */
        virtual bool recvTimingResp(PacketPtr pkt)
        { return xbar.recvTimingResp(pkt, id); }

        /**
         * When receiving a timing snoop request, pass it to the crossbar.
         */
        virtual void recvTimingSnoopReq(PacketPtr pkt)
        { return xbar.recvTimingSnoopReq(pkt, id); }

        /**
         * When receiving an atomic snoop request, pass it to the crossbar.
         */
        virtual Tick recvAtomicSnoop(PacketPtr pkt)
        { return xbar.recvAtomicSnoop(pkt, id); }

        /**
         * When receiving a functional snoop request, pass it to the crossbar.
         */
        virtual void recvFunctionalSnoop(PacketPtr pkt)
        { xbar.recvFunctionalSnoop(pkt, id); }

        /** When reciving a range change from the peer port (at id),
            pass it to the crossbar. */
        virtual void recvRangeChange()
        { xbar.recvRangeChange(id); }

        /** When reciving a retry from the peer port (at id),
            pass it to the crossbar. */
        virtual void recvReqRetry()
        { xbar.recvReqRetry(id); }

    };

    /**
     * Internal class to bridge between an incoming snoop response
     * from a slave port and forwarding it through an outgoing slave
     * port. It is effectively a dangling master port.
     */
    class SnoopRespPort : public MasterPort
    {

      private:

        /** The port which we mirror internally. */
        QueuedSlavePort& slavePort;

      public:

        /**
         * Create a snoop response port that mirrors a given slave port.
         */
        SnoopRespPort(QueuedSlavePort& slave_port, WirelessXBar& _xbar) :
            MasterPort(slave_port.name() + ".snoopRespPort", &_xbar),
            slavePort(slave_port) { }

        /**
         * Override the sending of retries and pass them on through
         * the mirrored slave port.
         */
        void sendRetryResp() {
            // forward it as a snoop response retry
            slavePort.sendRetrySnoopResp();
        }

        /**
         * Provided as necessary.
         */
        void recvReqRetry() { panic("SnoopRespPort should never see retry\n"); }

        /**
         * Provided as necessary.
         */
        bool recvTimingResp(PacketPtr pkt)
        {
            panic("SnoopRespPort should never see timing response\n");
            return false;
        }

    };

    std::vector<SnoopRespPort*> snoopRespPorts;

    std::vector<QueuedSlavePort*> snoopPorts;

    /**
     * Store the outstanding requests that we are expecting snoop
     * responses from so we can determine which snoop responses we
     * generated and which ones were merely forwarded.
     */
    std::unordered_set<RequestPtr> outstandingSnoop;

    /**
     * Store the outstanding cache maintenance that we are expecting
     * snoop responses from so we can determine when we received all
     * snoop responses and if any of the agents satisfied the request.
     */
    std::unordered_map<PacketId, PacketPtr> outstandingCMO;

    /**
     * Keep a pointer to the system to be allow to querying memory system
     * properties.
     */
    System *system;

    /** A snoop filter that tracks cache line residency and can restrict the
      * broadcast needed for probes.  NULL denotes an absent filter. */
    SnoopFilter *snoopFilter;

    /** Cycles of snoop response latency.*/
    const Cycles snoopResponseLatency;

    /** Is this crossbar the point of coherency? **/
    const bool pointOfCoherency;

    /** Is this crossbar the point of unification? **/
    const bool pointOfUnification;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

    /** Function called by the port when the crossbar is recieving a Timing
      request packet.*/
    bool recvTimingReq(PacketPtr pkt, PortID slave_port_id);

    /** Function called by the port when the crossbar is recieving a Timing
      response packet.*/
    bool recvTimingResp(PacketPtr pkt, PortID master_port_id);

    /** Function called by the port when the crossbar is recieving a timing
        snoop request.*/
    void recvTimingSnoopReq(PacketPtr pkt, PortID master_port_id);

    /** Function called by the port when the crossbar is recieving a timing
        snoop response.*/
    bool recvTimingSnoopResp(PacketPtr pkt, PortID slave_port_id);

    /** Timing function called by port when it is once again able to process
     * requests. */
    void recvReqRetry(PortID master_port_id);

    /**
     * Forward a timing packet to our snoopers, potentially excluding
     * one of the connected coherent masters to avoid sending a packet
     * back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     */
    void forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id) {
        forwardTiming(pkt, exclude_slave_port_id, snoopPorts);
    }

    /**
     * Forward a timing packet to a selected list of snoopers, potentially
     * excluding one of the connected coherent masters to avoid sending a packet
     * back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     * @param dests Vector of destination ports for the forwarded pkt
     */
    void forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id,
                       const std::vector<QueuedSlavePort*>& dests);

    /** Function called by the port when the crossbar is receiving a Atomic
      transaction.*/
    Tick recvAtomic(PacketPtr pkt, PortID slave_port_id);

    /** Function called by the port when the crossbar is receiving an
        atomic snoop transaction.*/
    Tick recvAtomicSnoop(PacketPtr pkt, PortID master_port_id);

    /**
     * Forward an atomic packet to our snoopers, potentially excluding
     * one of the connected coherent masters to avoid sending a packet
     * back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     *
     * @return a pair containing the snoop response and snoop latency
     */
    std::pair<MemCmd, Tick> forwardAtomic(PacketPtr pkt,
                                          PortID exclude_slave_port_id)
    {
        return forwardAtomic(pkt, exclude_slave_port_id, InvalidPortID,
                             snoopPorts);
    }

    /**
     * Forward an atomic packet to a selected list of snoopers, potentially
     * excluding one of the connected coherent masters to avoid sending a packet
     * back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     * @param source_master_port_id Id of the master port for snoops from below
     * @param dests Vector of destination ports for the forwarded pkt
     *
     * @return a pair containing the snoop response and snoop latency
     */
    std::pair<MemCmd, Tick> forwardAtomic(PacketPtr pkt,
                                          PortID exclude_slave_port_id,
                                          PortID source_master_port_id,
                                          const std::vector<QueuedSlavePort*>&
                                          dests);

    /** Function called by the port when the crossbar is recieving a Functional
        transaction.*/
    void recvFunctional(PacketPtr pkt, PortID slave_port_id);

    /** Function called by the port when the crossbar is recieving a functional
        snoop transaction.*/
    void recvFunctionalSnoop(PacketPtr pkt, PortID master_port_id);

    /**
     * Forward a functional packet to our snoopers, potentially
     * excluding one of the connected coherent masters to avoid
     * sending a packet back to where it came from.
     *
     * @param pkt Packet to forward
     * @param exclude_slave_port_id Id of slave port to exclude
     */
    void forwardFunctional(PacketPtr pkt, PortID exclude_slave_port_id);

    /**
     * Determine if the crossbar should sink the packet, as opposed to
     * forwarding it, or responding.
     */
    bool sinkPacket(const PacketPtr pkt) const;

    /**
     * Determine if the crossbar should forward the packet, as opposed to
     * responding to it.
     */
    bool forwardPacket(const PacketPtr pkt);

    /**
     * Determine if the packet's destination is the memory below
     *
     * The memory below is the destination for a cache mainteance
     * operation to the Point of Coherence/Unification if this is the
     * Point of Coherence/Unification.
     *
     * @param pkt The processed packet
     *
     * @return Whether the memory below is the destination for the packet
     */
    bool isDestination(const PacketPtr pkt) const
    {
        return (pkt->req->isToPOC() && pointOfCoherency) ||
            (pkt->req->isToPOU() && pointOfUnification);
    }

    /**
     * Calculate the timing parameters for the packet. Updates the
     * headerDelay and payloadDelay fields of the packet
     * object with the relative number of ticks required to transmit
     * the header and the payload, respectively.
     *
     * @param pkt Packet to populate with timings
     * @param header_delay Header delay to be added
     */
    void calcPacketWirelessTiming(PacketPtr pkt, Tick header_delay);


    /** 
     * Bandwidth (in ticks per byte) of the channels contained in this xbar
     * @TODO: different channels for the future
     */
    const double bandwidth;

    /** Stores which MAC protocol is being modelled */
    const Enums::MacProtocol macProto;

    /** Size of the collision retry slots for quantizing the retry windows */
    const Tick retrySlot;
    /** 
     * Maximum exponential for computing the number of slots in a retry 
     * window (maxSlots = 2^backoffCeil - 1)
     */
    const uint8_t backoffCeil;
    /**
     * Manages the random retransmission range. It is incremented every
     * collision and decremented every successful transmission
     */
    uint8_t backoffIndex;

    /** Holds the ID of the port who has the token for transmitting */
    uint16_t tokenID;
    /** Stores the offset for obtaining the token ID of slave ports */
    uint16_t idOffset;
    /** Stores the number of tokens (ports) that transmit */
    uint16_t numTokens;

    /** Track the state of the wireless channel */
    enum ChannelState { IDLE, BUSY };
    ChannelState channelState;

    /** 
     * A packet trying to be transmitted under exponential backoff protocol.
     * The packet might be deleted while waiting for retries, so we identify
     * them by source and destination ports
     */
    class ExpBackoffPkt {
      public:
        Tick tickStart;
        Tick tickSched;
        Tick txLength;
        uint8_t timesResched;
        bool overlap;
        Enums::LayerType layerType;
        Port* srcPort;
        PortID destPort;
        MasterID masterId;
        ExpBackoffPkt(Tick currTick, Enums::LayerType _layerType,
                      Port* _srcPort, PortID _destPort, MasterID _masterId)
            : tickStart(currTick), tickSched(0), txLength(0), timesResched(0),
              overlap(false), layerType(_layerType), srcPort(_srcPort),
              destPort(_destPort), masterId(_masterId)
        {}
    };
    typedef ExpBackoffPkt* ExpBackoffPktPtr;
    typedef std::list<ExpBackoffPkt> ExpBackoffPktList;
    /** List of collided packets to schedule randomly */
    ExpBackoffPktList pktToTransmitEBList;
    /** List of successfully scheduled packets waiting to transmit */
    ExpBackoffPktList pktReadyEBList;

    /** A packet waiting for its turn in token passing */
    class TokenPassPkt {
      public:
        Tick tickStart;
        Enums::LayerType layerType;
        Port* srcPort;
        PortID srcId;
        PortID destPort;
        MasterID masterId;
        TokenPassPkt(Tick currTick, Enums::LayerType _layerType, 
                      Port* _srcPort, PortID _srcId, PortID _destPort,
                      MasterID _masterId)
            : tickStart(currTick), layerType(_layerType), srcPort(_srcPort),
              srcId(_srcId), destPort(_destPort), masterId(_masterId)
        {}
    };
    typedef TokenPassPkt* TokenPassPktPtr;
    typedef std::list<TokenPassPkt> TokenPassPktList;
    /** List of packets waiting for holding the token */
    TokenPassPktList pktToTransmitTPList;

    /** 
     * This function checks if there has been collisions and either 
     * reschedules or calls the retry for finishing the transmission
     */
    void expBackoffCollisionCheck();

    /**
     * This function is in charge of changing the token holder
     */
    void tokenChange();

    /** Event called to check if the has been wireless collisions */
    EventFunctionWrapper collisionCheckEvent;

    /** Event called to change the token holder */
    EventFunctionWrapper tokenChangeEvent;

    /** @NOTE: SPM params needed? */

    /** Collided packet */
    // ProbePoints::PacketUPtr ppReqBlock;
    // ProbePoints::PacketUPtr ppRespBlock;  

    Stats::Scalar snoops;
    Stats::Scalar snoopTraffic;
    Stats::Distribution snoopFanout;
    
    /** Stats declaration for counting the succesful wireless transmissions */
    Stats::Vector wirelessTransmissions;
    /** Stats declaration for counting the number of wireless collisions */
    Stats::Vector wirelessCollisions;
    /** Stats declaration for monitoring the total wireless latency */
    Stats::Vector wirelessLatency;
    /** Stats declaration for monitoring the average wireless latency */
    Stats::Formula avgWirelessLatency;
    /** Stats declaration for monitoring the number of bytes transmitted */
    Stats::Vector bytesTransmitted;
    /** Stats declaration for monitoring the average transmission size */
    Stats::Formula avgBytesTransmitted;
    /** Auxiliar stats for checking the number of direct responses served */
    Stats::Scalar directResponses;

  public:

    virtual void init();

    WirelessXBar(const WirelessXBarParams *p);

    virtual ~WirelessXBar();

    /** Checks if two wireless transmissions overlap */
    bool checkOverlap(ExpBackoffPkt firstPkt, ExpBackoffPkt laterPkt);

    // Getters
    uint16_t getTokenID() { return tokenID; }
    PortID getDefaultPortID() { return defaultPortID; }

    bool channelStateIdle() { return channelState == IDLE; }
    void channelStateToBusy() { channelState = BUSY; }

    virtual void regStats();

    // void regProbePoints() override;
};

#endif //__MEM_WIRELESS_XBAR_HH__
