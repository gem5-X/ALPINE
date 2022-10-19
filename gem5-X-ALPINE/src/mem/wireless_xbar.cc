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
 * Definition of a wireless crossbar.
 */

#include "mem/wireless_xbar.hh"

#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "proto/packet.pb.h"
#include "debug/AddrRanges.hh"
#include "debug/Drain.hh"
#include "debug/WirelessXBar.hh"
#include "sim/system.hh"

WirelessXBar::WirelessXBar(const WirelessXBarParams *p)
    : BaseXBar(p), system(p->system), snoopFilter(p->snoop_filter),
      snoopResponseLatency(p->snoop_response_latency),
      pointOfCoherency(p->point_of_coherency),
      pointOfUnification(p->point_of_unification),
      bandwidth(p->bandwidth), macProto(p->mac_protocol),
      retrySlot(p->bandwidth * p->retry_slot_size),
      backoffCeil(p->backoff_ceil), backoffIndex(0),
      tokenID(0), idOffset(p->port_master_connection_count),
      numTokens(p->port_master_connection_count+p->port_slave_connection_count),
      channelState(IDLE), 
      collisionCheckEvent([this]{ expBackoffCollisionCheck(); }, name()),
      tokenChangeEvent([this]{ tokenChange(); }, name())
{
    // create the ports based on the size of the master and slave
    // vector ports, and the presence of the default port, the ports
    // are enumerated starting from zero
    for (int i = 0; i < p->port_master_connection_count; ++i) {
        std::string portName = csprintf("%s.master[%d]", name(), i);
        MasterPort* bp = new WirelessXBarMasterPort(portName, *this, i);
        masterPorts.push_back(bp);
        reqLayers.push_back(new ReqWirelessLayer(*bp, *this,
                                        csprintf(".reqLayer%d", i), macProto));
        snoopLayers.push_back(new SnoopRespWirelessLayer(*bp, *this,
                                      csprintf(".snoopLayer%d", i), macProto));
    }

    // see if we have a default slave device connected and if so add
    // our corresponding master port
    if (p->port_default_connection_count) {
        defaultPortID = masterPorts.size();
        std::string portName = name() + ".default";
        MasterPort* bp = new WirelessXBarMasterPort(portName, *this,
                                                   defaultPortID);
        masterPorts.push_back(bp);
        reqLayers.push_back(new ReqWirelessLayer(*bp, *this,
                                            csprintf(".reqLayer%d",
                                                defaultPortID), macProto));
        snoopLayers.push_back(new SnoopRespWirelessLayer(*bp, *this,
                                                 csprintf(".snoopLayer%d",
                                                    defaultPortID), macProto));
    }

    // create the slave ports, once again starting at zero
    for (int i = 0; i < p->port_slave_connection_count; ++i) {
        std::string portName = csprintf("%s.slave[%d]", name(), i);
        QueuedSlavePort* bp = new WirelessXBarSlavePort(portName, *this, i);
        slavePorts.push_back(bp);
        respLayers.push_back(new RespWirelessLayer(*bp, *this,
                                        csprintf(".respLayer%d", i), macProto));
        snoopRespPorts.push_back(new SnoopRespPort(*bp, *this));
    }

    DPRINTF(WirelessXBar, "Added layers and ports to Wireless XBar\n");
}

WirelessXBar::~WirelessXBar()
{
    for (auto l: reqLayers)
        delete l;
    for (auto l: respLayers)
        delete l;
    for (auto l: snoopLayers)
        delete l;
    for (auto p: snoopRespPorts)
        delete p;
}

void
WirelessXBar::init()
{
    BaseXBar::init();

    // iterate over our slave ports and determine which of our
    // neighbouring master ports are snooping and add them as snoopers
    for (const auto& p: slavePorts) {
        // check if the connected master port is snooping
        if (p->isSnooping()) {
            DPRINTF(AddrRanges, "Adding snooping master %s\n",
                    p->getMasterPort().name());
            snoopPorts.push_back(p);
        }
    }

    if (snoopPorts.empty())
        warn("WirelessXBar %s has no snooping ports attached!\n", name());

    // inform the snoop filter about the slave ports so it can create
    // its own internal representation
    if (snoopFilter)
        snoopFilter->setSlavePorts(slavePorts);
}

bool
WirelessXBar::recvTimingReq(PacketPtr pkt, PortID slave_port_id)
{   
    // determine the source port based on the id
    SlavePort *src_port = slavePorts[slave_port_id];

    // remember if the packet is an express snoop
    bool is_express_snoop = pkt->isExpressSnoop();
    bool cache_responding = pkt->cacheResponding();
    // for normal requests, going downstream, the express snoop flag
    // and the cache responding flag should always be the same
    assert(is_express_snoop == cache_responding);

    // determine the destination based on the destination address range
    AddrRange addr_range = RangeSize(pkt->getAddr(), pkt->getSize());
    PortID master_port_id = findPort(addr_range);

    // test if the crossbar accepts the packet (in exp. backoff after at least
    // the first theoretical delay), and exclude express snoops from the check
    if (!is_express_snoop && !reqLayers[master_port_id]->tryTiming(src_port,
                slave_port_id + idOffset, master_port_id, pkt, Enums::REQ)) {
        DPRINTF(WirelessXBar, "%s: src %s packet %s BUSY\n", __func__,
                src_port->name(), pkt->print());
        return false;
    }

    DPRINTF(WirelessXBar, "%s: src %s packet %s\n", __func__,
            src_port->name(), pkt->print());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // store the old header delay so we can restore it if needed
    Tick old_header_delay = pkt->headerDelay;

    // a request sees the frontend and forward latency
    Tick xbar_delay = (frontendLatency + forwardLatency) * clockPeriod();

    // set the packet header and payload delay
    // calcPacketTiming(pkt, xbar_delay); // wired version
    calcPacketWirelessTiming(pkt, xbar_delay);

    // @NOTE we won't use it for expBackoff, but maybe for others
    // determine how long to be crossbar layer is busy 
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    // is this the destination point for this packet? (e.g. true if
    // this xbar is the PoC for a cache maintenance operation to the
    // PoC) otherwise the destination is any cache that can satisfy
    // the request
    const bool is_destination = isDestination(pkt);

    const bool snoop_caches = !system->bypassCaches() &&
        pkt->cmd != MemCmd::WriteClean;
    if (snoop_caches) {
        assert(pkt->snoopDelay == 0);

        if (pkt->isClean() && !is_destination) {
            // before snooping we need to make sure that the memory
            // below is not busy and the cache clean request can be
            // forwarded to it
            if (!masterPorts[master_port_id]->tryTiming(pkt)) {
                DPRINTF(WirelessXBar, "%s: src %s packet %s RETRY\n", __func__,
                        src_port->name(), pkt->print());

                // update the layer state and schedule an idle event
                reqLayers[master_port_id]->failedTiming(src_port,
                                                        clockEdge(Cycles(1)));

                return false;
            }
        }


        // the packet is a memory-mapped request and should be
        // broadcasted to our snoopers but the source
        if (snoopFilter) {
            // check with the snoop filter where to forward this packet
            auto sf_res = snoopFilter->lookupRequest(pkt, *src_port);
            // the time required by a packet to be delivered through
            // the xbar has to be charged also with to lookup latency
            // of the snoop filter
            pkt->headerDelay += sf_res.second * clockPeriod();
            DPRINTF(WirelessXBar, "%s: src %s packet %s SF size: %i lat: %i\n",
                    __func__, src_port->name(), pkt->print(),
                    sf_res.first.size(), sf_res.second);

            if (pkt->isEviction()) {
                // for block-evicting packets, i.e. writebacks and
                // clean evictions, there is no need to snoop up, as
                // all we do is determine if the block is cached or
                // not, instead just set it here based on the snoop
                // filter result
                if (!sf_res.first.empty())
                    pkt->setBlockCached();
            } else {
                forwardTiming(pkt, slave_port_id, sf_res.first);
            }
        } else {
            forwardTiming(pkt, slave_port_id);
        }

        // add the snoop delay to our header delay, and then reset it
        pkt->headerDelay += pkt->snoopDelay;
        pkt->snoopDelay = 0;
    }

    // set up a sensible starting point
    bool success = true;

    // remember if the packet will generate a snoop response by
    // checking if a cache set the cacheResponding flag during the
    // snooping above
    const bool expect_snoop_resp = !cache_responding && pkt->cacheResponding();
    bool expect_response = pkt->needsResponse() && !pkt->cacheResponding();

    const bool sink_packet = sinkPacket(pkt);

    // in certain cases the crossbar is responsible for responding
    bool respond_directly = false;
    // store the original address as an address mapper could possibly
    // modify the address upon a sendTimingRequest
    const Addr addr(pkt->getAddr());
    if (sink_packet) {
        DPRINTF(WirelessXBar, "%s: Not forwarding %s\n", __func__,
                pkt->print());
    } else {
        // determine if we are forwarding the packet, or responding to
        // it
        if (forwardPacket(pkt)) {
            // if we are passing on, rather than sinking, a packet to
            // which an upstream cache has committed to responding,
            // the line was needs writable, and the responding only
            // had an Owned copy, so we need to immidiately let the
            // downstream caches know, bypass any flow control
            if (pkt->cacheResponding()) {
                pkt->setExpressSnoop();
            }

            // make sure that the write request (e.g., WriteClean)
            // will stop at the memory below if this crossbar is its
            // destination
            if (pkt->isWrite() && is_destination) {
                pkt->clearWriteThrough();
            }

            // since it is a normal request, attempt to send the packet
            success = masterPorts[master_port_id]->sendTimingReq(pkt);
        } else {
            // no need to forward, turn this packet around and respond
            // directly
            assert(pkt->needsResponse());

            respond_directly = true;
            assert(!expect_snoop_resp);
            expect_response = false;
        }
    }

    if (snoopFilter && snoop_caches) {
        // Let the snoop filter know about the success of the send operation
        snoopFilter->finishRequest(!success, addr, pkt->isSecure());
    }

    // check if we were successful in sending the packet onwards
    if (!success)  {
        // express snoops should never be forced to retry
        assert(!is_express_snoop);

        // restore the header delay
        pkt->headerDelay = old_header_delay;

        DPRINTF(WirelessXBar, "%s: src %s packet %s RETRY\n", __func__,
                src_port->name(), pkt->print());

        // update the layer state and schedule an idle event
        reqLayers[master_port_id]->failedTiming(src_port,
                                                clockEdge(Cycles(1)));
    } else {
        // express snoops currently bypass the crossbar state entirely
        if (!is_express_snoop) {
            // if this particular request will generate a snoop
            // response
            if (expect_snoop_resp) {
                // we should never have an exsiting request outstanding
                assert(outstandingSnoop.find(pkt->req) ==
                       outstandingSnoop.end());
                outstandingSnoop.insert(pkt->req);

                // basic sanity check on the outstanding snoops
                panic_if(outstandingSnoop.size() > 512,
                         "Outstanding snoop requests exceeded 512\n");
            }

            // remember where to route the normal response to
            if (expect_response || expect_snoop_resp) {
                assert(routeTo.find(pkt->req) == routeTo.end());
                routeTo[pkt->req] = slave_port_id;

                panic_if(routeTo.size() > 512,
                         "Routing table exceeds 512 packets\n");
            }

            // update the layer state and schedule an idle event
            reqLayers[master_port_id]->succeededTiming(packetFinishTime);
        }

        // stats updates only consider packets that were successfully sent
        pktCount[slave_port_id][master_port_id]++;
        pktSize[slave_port_id][master_port_id] += pkt_size;
        transDist[pkt_cmd]++;

        if (is_express_snoop) {
            snoops++;
            snoopTraffic += pkt_size;
        }
    }

    if (sink_packet)
        // queue the packet for deletion
        pendingDelete.reset(pkt);

    // normally we respond to the packet we just received if we need to
    PacketPtr rsp_pkt = pkt;
    PortID rsp_port_id = slave_port_id;

    // If this is the destination of the cache clean operation the
    // crossbar is responsible for responding. This crossbar will
    // respond when the cache clean is complete. A cache clean
    // is complete either:
    // * direcly, if no cache above had a dirty copy of the block
    //   as indicated by the satisfied flag of the packet, or
    // * when the crossbar has seen both the cache clean request
    //   (CleanSharedReq, CleanInvalidReq) and the corresponding
    //   write (WriteClean) which updates the block in the memory
    //   below.
    if (success &&
        ((pkt->isClean() && pkt->satisfied()) ||
         pkt->cmd == MemCmd::WriteClean) &&
        is_destination) {
        PacketPtr deferred_rsp = pkt->isWrite() ? nullptr : pkt;
        auto cmo_lookup = outstandingCMO.find(pkt->id);
        if (cmo_lookup != outstandingCMO.end()) {
            // the cache clean request has already reached this xbar
            respond_directly = true;
            if (pkt->isWrite()) {
                rsp_pkt = cmo_lookup->second;
                assert(rsp_pkt);

                // determine the destination
                const auto route_lookup = routeTo.find(rsp_pkt->req);
                assert(route_lookup != routeTo.end());
                rsp_port_id = route_lookup->second;
                assert(rsp_port_id != InvalidPortID);
                assert(rsp_port_id < respLayers.size());
                // remove the request from the routing table
                routeTo.erase(route_lookup);
            }
            outstandingCMO.erase(cmo_lookup);
        } else {
            respond_directly = false;
            outstandingCMO.emplace(pkt->id, deferred_rsp);
            if (!pkt->isWrite()) {
                assert(routeTo.find(pkt->req) == routeTo.end());
                routeTo[pkt->req] = slave_port_id;

                panic_if(routeTo.size() > 512,
                         "Routing table exceeds 512 packets\n");
            }
        }
    }

    if (respond_directly) { // @NOTE what to do with this? -> instead of sched
                            // the response directly, put in the waiting pkts
                            // list and sched the function handling it
                            // HOWEVER, I think it's not needed as there's no
                            // data and only happens after complete writeClean
                            // so we could have the wired approximation
        assert(rsp_pkt->needsResponse());
        assert(success);

        directResponses++;  // @NOTE seeing if this is significant

        rsp_pkt->makeResponse();

        if (snoopFilter && !system->bypassCaches()) {
            // let the snoop filter inspect the response and update its state
            snoopFilter->updateResponse(rsp_pkt, *slavePorts[rsp_port_id]);
        }

        // we send the response after the current packet, even if the
        // response is not for this packet (e.g. cache clean operation
        // where both the request and the write packet have to cross
        // the destination xbar before the response is sent.)
        Tick response_time = clockEdge() + pkt->headerDelay; 
        rsp_pkt->headerDelay = 0;

        slavePorts[rsp_port_id]->schedTimingResp(rsp_pkt, response_time);
    }
    return success;
}

bool
WirelessXBar::recvTimingResp(PacketPtr pkt, PortID master_port_id)
{
    // determine the source port based on the id
    MasterPort *src_port = masterPorts[master_port_id];

    // determine the destination
    const auto route_lookup = routeTo.find(pkt->req);
    assert(route_lookup != routeTo.end());
    const PortID slave_port_id = route_lookup->second;
    assert(slave_port_id != InvalidPortID);
    assert(slave_port_id < respLayers.size());

    // test if the crossbar should be considered occupied for the
    // current port
    if (!respLayers[slave_port_id]->tryTiming(src_port, 
                            master_port_id, slave_port_id, pkt, Enums::RESP)) {
        DPRINTF(WirelessXBar, "%s: src %s packet %s BUSY\n", __func__,
                src_port->name(), pkt->print());
        return false;
    }

    DPRINTF(WirelessXBar, "%s: src %s packet %s\n", __func__,
            src_port->name(), pkt->print());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // a response sees the response latency
    Tick xbar_delay = responseLatency * clockPeriod();

    // set the packet header and payload delay
    // calcPacketTiming(pkt, xbar_delay); // wired version
    calcPacketWirelessTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    if (snoopFilter && !system->bypassCaches()) {
        // let the snoop filter inspect the response and update its state
        snoopFilter->updateResponse(pkt, *slavePorts[slave_port_id]);
    }

    // send the packet through the destination slave port and pay for
    // any outstanding header delay
    Tick latency = pkt->headerDelay;
    pkt->headerDelay = 0;
    slavePorts[slave_port_id]->schedTimingResp(pkt, curTick() + latency);

    // remove the request from the routing table
    routeTo.erase(route_lookup);

    respLayers[slave_port_id]->succeededTiming(packetFinishTime);

    // stats updates
    pktCount[slave_port_id][master_port_id]++;
    pktSize[slave_port_id][master_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}

void
WirelessXBar::recvTimingSnoopReq(PacketPtr pkt, PortID master_port_id)
{
    DPRINTF(WirelessXBar, "%s: src %s packet %s\n", __func__,
            masterPorts[master_port_id]->name(), pkt->print());
    
    // update stats here as we know the forwarding will succeed
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    transDist[pkt->cmdToIndex()]++;
    snoops++;
    snoopTraffic += pkt_size;

    // we should only see express snoops from caches
    assert(pkt->isExpressSnoop());

    // set the packet header and payload delay, for now use forward latency
    // @todo Assess the choice of latency further
    calcPacketTiming(pkt, forwardLatency * clockPeriod());
    // @NOTE I won't use calcWirelessPacketTiming for it being atomic,
    // but will have to check when we check forwardTiming

    // remember if a cache has already committed to responding so we
    // can see if it changes during the snooping
    const bool cache_responding = pkt->cacheResponding();

    assert(pkt->snoopDelay == 0);

    if (snoopFilter) {
        // let the Snoop Filter work its magic and guide probing
        auto sf_res = snoopFilter->lookupSnoop(pkt);
        // the time required by a packet to be delivered through
        // the xbar has to be charged also with to lookup latency
        // of the snoop filter
        pkt->headerDelay += sf_res.second * clockPeriod();
        DPRINTF(WirelessXBar, "%s: src %s packet %s SF size: %i lat: %i\n",
                __func__, masterPorts[master_port_id]->name(), pkt->print(),
                sf_res.first.size(), sf_res.second);

        // forward to all snoopers
        forwardTiming(pkt, InvalidPortID, sf_res.first);
    } else {
        forwardTiming(pkt, InvalidPortID);
    }

    // add the snoop delay to our header delay, and then reset it
    pkt->headerDelay += pkt->snoopDelay;
    pkt->snoopDelay = 0;

    // if we can expect a response, remember how to route it
    if (!cache_responding && pkt->cacheResponding()) {
        assert(routeTo.find(pkt->req) == routeTo.end());
        routeTo[pkt->req] = master_port_id;
    }

    // a snoop request came from a connected slave device (one of
    // our master ports), and if it is not coming from the slave
    // device responsible for the address range something is
    // wrong, hence there is nothing further to do as the packet
    // would be going back to where it came from
    AddrRange addr_range M5_VAR_USED =
        RangeSize(pkt->getAddr(), pkt->getSize());
    assert(findPort(addr_range) == master_port_id);
}

bool
WirelessXBar::recvTimingSnoopResp(PacketPtr pkt, PortID slave_port_id)
{
    // determine the source port based on the id
    SlavePort* src_port = slavePorts[slave_port_id];

    // get the destination
    const auto route_lookup = routeTo.find(pkt->req);
    assert(route_lookup != routeTo.end());
    const PortID dest_port_id = route_lookup->second;
    assert(dest_port_id != InvalidPortID);

    // determine if the response is from a snoop request we
    // created as the result of a normal request (in which case it
    // should be in the outstandingSnoop), or if we merely forwarded
    // someone else's snoop request
    const bool forwardAsSnoop = outstandingSnoop.find(pkt->req) ==
        outstandingSnoop.end();

    // test if the crossbar should be considered occupied for the
    // current port, note that the check is bypassed if the response
    // is being passed on as a normal response since this is occupying
    // the response layer rather than the snoop response layer
    if (forwardAsSnoop) {
        assert(dest_port_id < snoopLayers.size());
        if (!snoopLayers[dest_port_id]->tryTiming(src_port,
                slave_port_id + idOffset, dest_port_id, pkt, Enums::SNOOP)) {
            DPRINTF(WirelessXBar, "%s: src %s packet %s BUSY\n", __func__,
                    src_port->name(), pkt->print());
            return false;
        }
    } else {
        // get the master port that mirrors this slave port internally
        MasterPort* snoop_port = snoopRespPorts[slave_port_id];
        assert(dest_port_id < respLayers.size());
        if (!respLayers[dest_port_id]->tryTiming(snoop_port, 
                slave_port_id + idOffset, dest_port_id, pkt, Enums::RESP)) {
            DPRINTF(WirelessXBar, "%s: src %s packet %s BUSY\n", __func__,
                    snoop_port->name(), pkt->print());
            return false;
        }
    }

    DPRINTF(WirelessXBar, "%s: src %s packet %s\n", __func__,
            src_port->name(), pkt->print());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // responses are never express snoops
    assert(!pkt->isExpressSnoop());

    // a snoop response sees the snoop response latency, and if it is
    // forwarded as a normal response, the response latency
    Tick xbar_delay =
        (forwardAsSnoop ? snoopResponseLatency : responseLatency) *
        clockPeriod();

    // set the packet header and payload delay
    // calcPacketTiming(pkt, xbar_delay); // wired version
    calcPacketWirelessTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    // forward it either as a snoop response or a normal response
    if (forwardAsSnoop) {
        // this is a snoop response to a snoop request we forwarded,
        // e.g. coming from the L1 and going to the L2, and it should
        // be forwarded as a snoop response

        if (snoopFilter) {
            // update the probe filter so that it can properly track the line
            snoopFilter->updateSnoopForward(pkt, *slavePorts[slave_port_id],
                                            *masterPorts[dest_port_id]);
        }

        bool success M5_VAR_USED =
            masterPorts[dest_port_id]->sendTimingSnoopResp(pkt);
        pktCount[slave_port_id][dest_port_id]++;
        pktSize[slave_port_id][dest_port_id] += pkt_size;
        assert(success);

        snoopLayers[dest_port_id]->succeededTiming(packetFinishTime);
    } else {
        // we got a snoop response on one of our slave ports,
        // i.e. from a coherent master connected to the crossbar, and
        // since we created the snoop request as part of recvTiming,
        // this should now be a normal response again
        outstandingSnoop.erase(pkt->req);

        // this is a snoop response from a coherent master, hence it
        // should never go back to where the snoop response came from,
        // but instead to where the original request came from
        assert(slave_port_id != dest_port_id);

        if (snoopFilter) {
            // update the probe filter so that it can properly track the line
            snoopFilter->updateSnoopResponse(pkt, *slavePorts[slave_port_id],
                                    *slavePorts[dest_port_id]);
        }

        DPRINTF(WirelessXBar, "%s: src %s packet %s FWD RESP\n", __func__,
                src_port->name(), pkt->print());

        // as a normal response, it should go back to a master through
        // one of our slave ports, we also pay for any outstanding
        // header latency
        Tick latency = pkt->headerDelay;
        pkt->headerDelay = 0;
        slavePorts[dest_port_id]->schedTimingResp(pkt, curTick() + latency);

        respLayers[dest_port_id]->succeededTiming(packetFinishTime);
    }

    // remove the request from the routing table
    routeTo.erase(route_lookup);

    // stats updates
    transDist[pkt_cmd]++;
    snoops++;
    snoopTraffic += pkt_size;

    return true;
}

// @NOTE: right now no modification (atomic, non-wireless snoopResp from higher
// caches), But will have to think about this in the future
void
WirelessXBar::forwardTiming(PacketPtr pkt, PortID exclude_slave_port_id,
                           const std::vector<QueuedSlavePort*>& dests)
{
    DPRINTF(WirelessXBar, "%s for %s\n", __func__, pkt->print());

    // snoops should only happen if the system isn't bypassing caches
    assert(!system->bypassCaches());

    unsigned fanout = 0;

    for (const auto& p: dests) {
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (exclude_slave_port_id == InvalidPortID ||
            p->getId() != exclude_slave_port_id) {
            // cache is not allowed to refuse snoop
            p->sendTimingSnoopReq(pkt);
            fanout++;
        }
    }

    // Stats for fanout of this forward operation
    snoopFanout.sample(fanout);
}

void
WirelessXBar::recvReqRetry(PortID master_port_id)
{
    // responses and snoop responses never block on forwarding them,
    // so the retry will always be coming from a port to which we
    // tried to forward a request
    reqLayers[master_port_id]->recvRetry();
}

Tick
WirelessXBar::recvAtomic(PacketPtr pkt, PortID slave_port_id)
{
    DPRINTF(WirelessXBar, "%s: src %s packet %s\n", __func__,
            slavePorts[slave_port_id]->name(), pkt->print());

    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    MemCmd snoop_response_cmd = MemCmd::InvalidCmd;
    Tick snoop_response_latency = 0;

    // is this the destination point for this packet? (e.g. true if
    // this xbar is the PoC for a cache maintenance operation to the
    // PoC) otherwise the destination is any cache that can satisfy
    // the request
    const bool is_destination = isDestination(pkt);

    const bool snoop_caches = !system->bypassCaches() &&
        pkt->cmd != MemCmd::WriteClean;
    if (snoop_caches) {
        // forward to all snoopers but the source
        std::pair<MemCmd, Tick> snoop_result;
        if (snoopFilter) {
            // check with the snoop filter where to forward this packet
            auto sf_res =
                snoopFilter->lookupRequest(pkt, *slavePorts[slave_port_id]);
            snoop_response_latency += sf_res.second * clockPeriod();
            DPRINTF(WirelessXBar, "%s: src %s packet %s SF size: %i lat: %i\n",
                    __func__, slavePorts[slave_port_id]->name(), pkt->print(),
                    sf_res.first.size(), sf_res.second);

            // let the snoop filter know about the success of the send
            // operation, and do it even before sending it onwards to
            // avoid situations where atomic upward snoops sneak in
            // between and change the filter state
            snoopFilter->finishRequest(false, pkt->getAddr(), pkt->isSecure());

            if (pkt->isEviction()) {
                // for block-evicting packets, i.e. writebacks and
                // clean evictions, there is no need to snoop up, as
                // all we do is determine if the block is cached or
                // not, instead just set it here based on the snoop
                // filter result
                if (!sf_res.first.empty())
                    pkt->setBlockCached();
            } else {
                snoop_result = forwardAtomic(pkt, slave_port_id, InvalidPortID,
                                             sf_res.first);
            }
        } else {
            snoop_result = forwardAtomic(pkt, slave_port_id);
        }
        snoop_response_cmd = snoop_result.first;
        snoop_response_latency += snoop_result.second;
    }

    // set up a sensible default value
    Tick response_latency = 0;

    const bool sink_packet = sinkPacket(pkt);

    // even if we had a snoop response, we must continue and also
    // perform the actual request at the destination
    AddrRange addr_range = RangeSize(pkt->getAddr(), pkt->getSize());
    PortID master_port_id = findPort(addr_range);

    if (sink_packet) {
        DPRINTF(WirelessXBar, "%s: Not forwarding %s\n", __func__,
                pkt->print());
    } else {
        if (forwardPacket(pkt)) {
            // make sure that the write request (e.g., WriteClean)
            // will stop at the memory below if this crossbar is its
            // destination
            if (pkt->isWrite() && is_destination) {
                pkt->clearWriteThrough();
            }

            // forward the request to the appropriate destination
            response_latency = masterPorts[master_port_id]->sendAtomic(pkt);
        } else {
            // if it does not need a response we sink the packet above
            assert(pkt->needsResponse());

            pkt->makeResponse();
        }
    }

    // stats updates for the request
    pktCount[slave_port_id][master_port_id]++;
    pktSize[slave_port_id][master_port_id] += pkt_size;
    transDist[pkt_cmd]++;


    // if lower levels have replied, tell the snoop filter
    if (!system->bypassCaches() && snoopFilter && pkt->isResponse()) {
        snoopFilter->updateResponse(pkt, *slavePorts[slave_port_id]);
    }

    // if we got a response from a snooper, restore it here
    if (snoop_response_cmd != MemCmd::InvalidCmd) {
        // no one else should have responded
        assert(!pkt->isResponse());
        pkt->cmd = snoop_response_cmd;
        response_latency = snoop_response_latency;
    }

    // If this is the destination of the cache clean operation the
    // crossbar is responsible for responding. This crossbar will
    // respond when the cache clean is complete. An atomic cache clean
    // is complete when the crossbars receives the cache clean
    // request (CleanSharedReq, CleanInvalidReq), as either:
    // * no cache above had a dirty copy of the block as indicated by
    //   the satisfied flag of the packet, or
    // * the crossbar has already seen the corresponding write
    //   (WriteClean) which updates the block in the memory below.
    if (pkt->isClean() && isDestination(pkt) && pkt->satisfied()) {
        auto it = outstandingCMO.find(pkt->id);
        assert(it != outstandingCMO.end());
        // we are responding right away
        outstandingCMO.erase(it);
    } else if (pkt->cmd == MemCmd::WriteClean && isDestination(pkt)) {
        // if this is the destination of the operation, the xbar
        // sends the responce to the cache clean operation only
        // after having encountered the cache clean request
        auto M5_VAR_USED ret = outstandingCMO.emplace(pkt->id, nullptr);
        // in atomic mode we know that the WriteClean packet should
        // precede the clean request
        assert(ret.second);
    }

    // add the response data
    if (pkt->isResponse()) {
        pkt_size = pkt->hasData() ? pkt->getSize() : 0;
        pkt_cmd = pkt->cmdToIndex();

        // stats updates
        pktCount[slave_port_id][master_port_id]++;
        pktSize[slave_port_id][master_port_id] += pkt_size;
        transDist[pkt_cmd]++;
    }

    // @todo: Not setting header time
    pkt->payloadDelay = response_latency;
    return response_latency;
}

Tick
WirelessXBar::recvAtomicSnoop(PacketPtr pkt, PortID master_port_id)
{
    DPRINTF(WirelessXBar, "%s: src %s packet %s\n", __func__,
            masterPorts[master_port_id]->name(), pkt->print());

    // add the request snoop data
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    snoops++;
    snoopTraffic += pkt_size;

    // forward to all snoopers
    std::pair<MemCmd, Tick> snoop_result;
    Tick snoop_response_latency = 0;
    if (snoopFilter) {
        auto sf_res = snoopFilter->lookupSnoop(pkt);
        snoop_response_latency += sf_res.second * clockPeriod();
        DPRINTF(WirelessXBar, "%s: src %s packet %s SF size: %i lat: %i\n",
                __func__, masterPorts[master_port_id]->name(), pkt->print(),
                sf_res.first.size(), sf_res.second);
        snoop_result = forwardAtomic(pkt, InvalidPortID, master_port_id,
                                     sf_res.first);
    } else {
        snoop_result = forwardAtomic(pkt, InvalidPortID);
    }
    MemCmd snoop_response_cmd = snoop_result.first;
    snoop_response_latency += snoop_result.second;

    if (snoop_response_cmd != MemCmd::InvalidCmd)
        pkt->cmd = snoop_response_cmd;

    // add the response snoop data
    if (pkt->isResponse()) {
        snoops++;
    }

    // @todo: Not setting header time
    pkt->payloadDelay = snoop_response_latency;
    return snoop_response_latency;
}

std::pair<MemCmd, Tick>
WirelessXBar::forwardAtomic(PacketPtr pkt, PortID exclude_slave_port_id,
                           PortID source_master_port_id,
                           const std::vector<QueuedSlavePort*>& dests)
{
    // the packet may be changed on snoops, record the original
    // command to enable us to restore it between snoops so that
    // additional snoops can take place properly
    MemCmd orig_cmd = pkt->cmd;
    MemCmd snoop_response_cmd = MemCmd::InvalidCmd;
    Tick snoop_response_latency = 0;

    // snoops should only happen if the system isn't bypassing caches
    assert(!system->bypassCaches());

    unsigned fanout = 0;

    for (const auto& p: dests) {
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (exclude_slave_port_id != InvalidPortID &&
            p->getId() == exclude_slave_port_id)
            continue;

        Tick latency = p->sendAtomicSnoop(pkt);
        fanout++;

        // in contrast to a functional access, we have to keep on
        // going as all snoopers must be updated even if we get a
        // response
        if (!pkt->isResponse())
            continue;

        // response from snoop agent
        assert(pkt->cmd != orig_cmd);
        assert(pkt->cacheResponding());
        // should only happen once
        assert(snoop_response_cmd == MemCmd::InvalidCmd);
        // save response state
        snoop_response_cmd = pkt->cmd;
        snoop_response_latency = latency;

        if (snoopFilter) {
            // Handle responses by the snoopers and differentiate between
            // responses to requests from above and snoops from below
            if (source_master_port_id != InvalidPortID) {
                // Getting a response for a snoop from below
                assert(exclude_slave_port_id == InvalidPortID);
                snoopFilter->updateSnoopForward(pkt, *p,
                             *masterPorts[source_master_port_id]);
            } else {
                // Getting a response for a request from above
                assert(source_master_port_id == InvalidPortID);
                snoopFilter->updateSnoopResponse(pkt, *p,
                             *slavePorts[exclude_slave_port_id]);
            }
        }
        // restore original packet state for remaining snoopers
        pkt->cmd = orig_cmd;
    }

    // Stats for fanout
    snoopFanout.sample(fanout);

    // the packet is restored as part of the loop and any potential
    // snoop response is part of the returned pair
    return std::make_pair(snoop_response_cmd, snoop_response_latency);
}

void
WirelessXBar::recvFunctional(PacketPtr pkt, PortID slave_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(WirelessXBar, "%s: src %s packet %s\n", __func__,
                slavePorts[slave_port_id]->name(), pkt->print());
    }

    if (!system->bypassCaches()) {
        // forward to all snoopers but the source
        forwardFunctional(pkt, slave_port_id);
    }

    // there is no need to continue if the snooping has found what we
    // were looking for and the packet is already a response
    if (!pkt->isResponse()) {
        // since our slave ports are queued ports we need to check them as well
        for (const auto& p : slavePorts) {
            // if we find a response that has the data, then the
            // downstream caches/memories may be out of date, so simply stop
            // here
            if (p->trySatisfyFunctional(pkt)) {
                if (pkt->needsResponse())
                    pkt->makeResponse();
                return;
            }
        }

        PortID dest_id = findPort(RangeSize(pkt->getAddr(), pkt->getSize()));

        masterPorts[dest_id]->sendFunctional(pkt);
    }
}

void
WirelessXBar::recvFunctionalSnoop(PacketPtr pkt, PortID master_port_id)
{
    if (!pkt->isPrint()) {
        // don't do DPRINTFs on PrintReq as it clutters up the output
        DPRINTF(WirelessXBar, "%s: src %s packet %s\n", __func__,
                masterPorts[master_port_id]->name(), pkt->print());
    }

    for (const auto& p : slavePorts) {
        if (p->trySatisfyFunctional(pkt)) {
            if (pkt->needsResponse())
                pkt->makeResponse();
            return;
        }
    }

    // forward to all snoopers
    forwardFunctional(pkt, InvalidPortID);
}

void
WirelessXBar::forwardFunctional(PacketPtr pkt, PortID exclude_slave_port_id)
{
    // snoops should only happen if the system isn't bypassing caches
    assert(!system->bypassCaches());

    for (const auto& p: snoopPorts) {
        // we could have gotten this request from a snooping master
        // (corresponding to our own slave port that is also in
        // snoopPorts) and should not send it back to where it came
        // from
        if (exclude_slave_port_id == InvalidPortID ||
            p->getId() != exclude_slave_port_id)
            p->sendFunctionalSnoop(pkt);

        // if we get a response we are done
        if (pkt->isResponse()) {
            break;
        }
    }
}

bool
WirelessXBar::sinkPacket(const PacketPtr pkt) const
{
    // we can sink the packet if:
    // 1) the crossbar is the point of coherency, and a cache is
    //    responding after being snooped
    // 2) the crossbar is the point of coherency, and the packet is a
    //    coherency packet (not a read or a write) that does not
    //    require a response
    // 3) this is a clean evict or clean writeback, but the packet is
    //    found in a cache above this crossbar
    // 4) a cache is responding after being snooped, and the packet
    //    either does not need the block to be writable, or the cache
    //    that has promised to respond (setting the cache responding
    //    flag) is providing writable and thus had a Modified block,
    //    and no further action is needed
    return (pointOfCoherency && pkt->cacheResponding()) ||
        (pointOfCoherency && !(pkt->isRead() || pkt->isWrite()) &&
         !pkt->needsResponse()) ||
        (pkt->isCleanEviction() && pkt->isBlockCached()) ||
        (pkt->cacheResponding() &&
         (!pkt->needsWritable() || pkt->responderHadWritable()));
}

bool
WirelessXBar::forwardPacket(const PacketPtr pkt)
{
    // we are forwarding the packet if:
    // 1) this is a cache clean request to the PoU/PoC and this
    //    crossbar is above the PoU/PoC
    // 2) this is a read or a write
    // 3) this crossbar is above the point of coherency
    if (pkt->isClean()) {
        return !isDestination(pkt);
    }
    return pkt->isRead() || pkt->isWrite() || !pointOfCoherency;
}

void
WirelessXBar::calcPacketWirelessTiming(PacketPtr pkt, Tick header_delay)
{
    // the crossbar will be called at a time that is not necessarily
    // coinciding with its own clock, so start by determining how long
    // until the next clock edge (could be zero)
    Tick offset = clockEdge() - curTick();

    // the header delay depends on the path through the crossbar, and
    // we therefore rely on the caller to provide the actual
    // value
    pkt->headerDelay += offset + header_delay;

    // note that we add the header delay to the existing value, and
    // align it to the crossbar clock

    if (pkt->hasData()) {
        // Check protocol used
        switch (macProto) {
            case Enums::exp_backoff:    // Exponential backoff
                // @NOTE 0 for now, maybe later we do want to do 
                // max between payloadDelay - wirelessDelay and 0
                pkt->payloadDelay = 0;
            break;
            case Enums::token_pass:
                pkt->payloadDelay = std::max<Tick>(pkt->payloadDelay,
                                                   pkt->getSize() * bandwidth);      
            break;
            default:
                pkt->payloadDelay = 0; // @NOTE empty for the moment
            break;
        }
    }
}

bool
WirelessXBar::checkOverlap(ExpBackoffPkt firstPkt, ExpBackoffPkt laterPkt)
{
    Tick laterTxStart = laterPkt.tickSched - laterPkt.txLength;
    return laterTxStart < firstPkt.tickSched;
}

void
WirelessXBar::expBackoffCollisionCheck()
{
    // Signals if the overlap is an old collision or a new one
    bool newOverlap = false;

    // Get first packet from the transmission list and erase from the list
    ExpBackoffPkt curPkt = pktToTransmitEBList.front();

    DPRINTF(WirelessXBar, "%s: starting collision check to port %d, "
        "transmission rescheduled %d times\n", __func__, curPkt.destPort,
        curPkt.timesResched);

    assert(curPkt.tickSched == curTick());

    pktToTransmitEBList.pop_front();

    // Loop through the rest checking if overlap
    bool overlap = curPkt.overlap;   
    auto i = pktToTransmitEBList.begin();
    while (i != pktToTransmitEBList.end()) {
        // If overlap, mark as such in the pktToTransmitEBList 
        if (checkOverlap(curPkt, *i)) {
            newOverlap = !i->overlap; // if it wasn't overlap before
            i->overlap = true;
            overlap = true;
        }
        i++;
    }

    if (overlap) { // Increase backoff index and reschedule only current packet

        if (newOverlap) {
            backoffIndex = std::min(uint8_t (backoffIndex + 1), backoffCeil);
        }

        DPRINTF(WirelessXBar, 
                "%s: overlap, rescheduling transmission, backoff index is %d\n",
                __func__, backoffIndex);

        // Compute size of retry windows through exponential backoff and
        // obtain a random number of slots within it
        uint16_t rnd_slots_delay = random_mt.random(0,(1 << backoffIndex)-1);

        // Assign tentative end transmission time
        curPkt.tickSched = curTick() + rnd_slots_delay * retrySlot
                                + curPkt.txLength;
        curPkt.timesResched++;

        // Register new collision
        wirelessCollisions[curPkt.masterId]++;
        // const ProbePoints::PacketInfo pkt_info(curPkt.pkt);
        // if (curPkt.pkt->isRequest()) {
        //     ppReqBlock->notify(pkt_info);
        // } else if (curPkt.pkt->isResponse()) {
        //     ppRespBlock->notify(pkt_info);
        // }

        // Toggle the overlap flag of the packet for next transmission
        curPkt.overlap = false;

        // Insert the ExpBackoffPkt in the pktToTransmitEBList
        if (pktToTransmitEBList.empty()) {
            pktToTransmitEBList.push_front(curPkt);
        } else {
            // Look for the right place to insert so that packets are 
            // ordered by their scheduled tick
            auto i = pktToTransmitEBList.end();
            --i;
            while (i != pktToTransmitEBList.begin() &&
                curPkt.tickSched < i->tickSched)
                --i;
            
            if (i == pktToTransmitEBList.begin() &&
                curPkt.tickSched < i->tickSched) {
                // inserts the element before the position pointed
                // to by the iterator, so before the beginning
                pktToTransmitEBList.insert(i, curPkt);
            } else {
                // inserts the element before the position pointed
                // to by the iterator, so advance it one step
                pktToTransmitEBList.insert(++i, curPkt);
            }
        }

    } else {

        backoffIndex = std::max(backoffIndex - 1, 0);

        DPRINTF(WirelessXBar,
                "%s: no overlap, finishing transmission, backoff index is %d\n",
                __func__, backoffIndex);

        // Push in the list of packets successfully scheduled
        pktReadyEBList.push_front(curPkt);
    
        // Register wireless transmission latency
        wirelessLatency[curPkt.masterId] += curPkt.tickSched - curPkt.tickStart;

        // Send retry req to port, which will be finally served
        switch (curPkt.layerType) {
            case Enums::REQ:
                reqLayers[curPkt.destPort]->triggerSendRetry(curPkt.srcPort);
            break;
            case Enums::RESP:
                respLayers[curPkt.destPort]->triggerSendRetry(curPkt.srcPort);
            break;
            case Enums::SNOOP:
                snoopLayers[curPkt.destPort]->triggerSendRetry(curPkt.srcPort);
            break;
            default:    break;
        }
    }
    
    if (!pktToTransmitEBList.empty()) {
        // Reschedule the collisionCheckEvent at the next transmission end
        reschedule(collisionCheckEvent, pktToTransmitEBList.begin()->tickSched,
                    true);
    }
}

void
WirelessXBar::tokenChange()
{
    // Free the wireless channel
    channelState = IDLE;

    // Advance token holder
    tokenID = (tokenID + 1) % numTokens;

    // Schedule next token change in case of silent cycle
    // It's done before sending retries, in case they transmit and reschedule
    reschedule(tokenChangeEvent, nextCycle(), true);

    // Send retry req to the holder of the token
    auto i = pktToTransmitTPList.begin();
    while (i->srcId != tokenID && i != pktToTransmitTPList.end()) {
        i++;
    }

    if (i != pktToTransmitTPList.end()) {
        switch (i->layerType) {
            case Enums::REQ:
                reqLayers[i->destPort]->triggerSendRetry(i->srcPort);
            break;
            case Enums::RESP:
                respLayers[i->destPort]->triggerSendRetry(i->srcPort);
            break;
            case Enums::SNOOP:
                snoopLayers[i->destPort]->triggerSendRetry(i->srcPort);
            break;
            default:    break;
        }
    }
}

void
WirelessXBar::regStats()
{
    // register the stats of the base class and our layers
    BaseXBar::regStats();
    for (auto l: reqLayers)
        l->regStats();
    for (auto l: respLayers)
        l->regStats();
    for (auto l: snoopLayers)
        l->regStats();

    using namespace Stats;

    snoops
        .name(name() + ".snoops")
        .desc("Total snoops (count)")
    ;

    snoopTraffic
        .name(name() + ".snoopTraffic")
        .desc("Total snoop traffic (bytes)")
    ;

    snoopFanout
        .init(0, snoopPorts.size(), 1)
        .name(name() + ".snoop_fanout")
        .desc("Request fanout histogram")
    ;

    wirelessTransmissions
        .init(system->maxMasters())
        .name(name() + ".wireless_transmissions")
        .desc("Number of successful wireless transmissions")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system->maxMasters(); i++) {
        wirelessTransmissions.subname(i, system->getMasterName(i));
    }

    wirelessCollisions
        .init(system->maxMasters())
        .name(name() + ".wireless_collisions")
        .desc("Number of collided wireless packets (in Token Passing, "
             "if latency > than worst ideal case")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system->maxMasters(); i++) {
        wirelessCollisions.subname(i, system->getMasterName(i));
    }

    bytesTransmitted
        .init(system->maxMasters())
        .name(name() + ".bytes_transmitted")
        .desc("total number of bytes transmitted wirelessly")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i< system->maxMasters(); i++) {
        bytesTransmitted.subname(i, system->getMasterName(i));
    }

    avgBytesTransmitted
        .name(name() + ".avg_bytes_transmitted")
        .desc("average size of the wireless transmissions in bytes")
        .flags(total | nozero | nonan)
        ;
    avgBytesTransmitted = bytesTransmitted / wirelessTransmissions;

    for (int i = 0; i< system->maxMasters(); i++) {
        avgBytesTransmitted.subname(i, system->getMasterName(i));
    }

    wirelessLatency
        .init(system->maxMasters())
        .name(name() + ".wireless_latency")
        .desc("total latency of wireless transmission in number of cycles")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i< system->maxMasters(); i++) {
        wirelessLatency.subname(i, system->getMasterName(i));
    }

    avgWirelessLatency
        .name(name() + ".avg_wireless_latency")
        .desc("average latency of wireless transmission per byte " 
              "in number of cycles")
        .flags(total | nozero | nonan)
        ;
    avgWirelessLatency = wirelessLatency / bytesTransmitted;

    for (int i = 0; i< system->maxMasters(); i++) {
        avgWirelessLatency.subname(i, system->getMasterName(i));
    }

    directResponses
        .name(name() + ".directResponses")
        .desc("auxiliar stat for checking how reasonable wireless model is")
        ;
}

// void
// WirelessXBar::regProbePoints()
// {
//     ppReqBlock.reset(new ProbePoints::Packet(getProbeManager(), "PktRequest"));
//     ppRespBlock.reset(new ProbePoints::Packet(getProbeManager(), "PktResponse"));
// }

WirelessXBar *
WirelessXBarParams::create()
{
    return new WirelessXBar(this);
}

template <typename SrcType, typename DstType>
WirelessXBar::WirelessLayer<SrcType,DstType>::WirelessLayer(DstType& _port,
                                                WirelessXBar& _xbar,
                                                const std::string& _name,
                                                Enums::MacProtocol _macProto) :
    port(_port), xbar(_xbar), _name(_name), macProto(_macProto), state(IDLE),
    releaseEvent([this]{ releaseLayer(); }, name())
{
}

template <typename SrcType, typename DstType>
bool
WirelessXBar::WirelessLayer<SrcType,DstType>::tryTiming(SrcType* src_port,
                                                    PortID src_id,
                                                    PortID dest_port,
                                                    PacketPtr pkt,
                                                    Enums::LayerType layerType)
{
    state = BUSY;

    // Check protocol used
    switch (macProto) {
        case Enums::exp_backoff:    // Exponential backoff
        {
            // Check if it's the first time we see this pkt, 
            // or if it's a retry from a previous deny
            if (!waitingForLayer.empty()) {
                auto waitingPkt = waitingForLayer.begin();
                while (*waitingPkt != src_port && 
                        waitingPkt != waitingForLayer.end()) {
                    ++waitingPkt;
                }
                if (waitingPkt != waitingForLayer.end()) {
                    // Already scheduled at least once
                    // Remove from waitingForLayer
                    waitingPkt = waitingForLayer.erase(waitingPkt);

                    // Find the packet in the pktToTransmitEBList
                    auto i = xbar.pktReadyEBList.begin();
                    while (i->srcPort != src_port && i != xbar.pktReadyEBList.end())
                        ++i;
                    
                    panic_if(i == xbar.pktReadyEBList.end(),
                            "Packet to be transmitted is not found in the list\n");

                    // Track layer occupancy
                    occupancy += i->txLength;
                    xbar.wirelessTransmissions[pkt->req->masterId()]++;

                    // Register transmitted bytes
                    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
                    xbar.bytesTransmitted[pkt->req->masterId()] += pkt_size;

                    // Remove from pktToTransmitEBList
                    i = xbar.pktReadyEBList.erase(i); // Erase i, get next element

                    return true;
                }
            }

            // First time seen it, so schedule tentative transmission
            
            // Signal that we're waiting for the end of the transmission
            waitingForLayer.push_back(src_port);

            // Compute when transmission should end if no collisions
            ExpBackoffPktPtr curPkt = new ExpBackoffPkt(curTick(),
                    layerType, src_port, dest_port, pkt->req->masterId());
            // Compute transmission length. If no payload, assume 1B
            curPkt->txLength = pkt->hasData() ? 
                pkt->getSize() * xbar.bandwidth : xbar.bandwidth;

            // Compute size of retry windows through exponential backoff and
            // obtain a random number of slots within it
            uint16_t rnd_slots_delay = 
                            random_mt.random(0,(1 << xbar.backoffIndex)-1);

            // Assign tentative end transmission time
            curPkt->tickSched = curTick() + rnd_slots_delay * xbar.retrySlot
                                    + curPkt->txLength;

            // Insert the ExpBackoffPkt in the pktToTransmitEBList
            if (xbar.pktToTransmitEBList.empty()) {
                xbar.pktToTransmitEBList.push_front(*curPkt);
            } else {
                // Look for the right place to insert so that packets are 
                // ordered by their scheduled tick
                auto i = xbar.pktToTransmitEBList.end();
                --i;
                while (i != xbar.pktToTransmitEBList.begin() &&
                    curPkt->tickSched < i->tickSched)
                    --i;
                
                if (i == xbar.pktToTransmitEBList.begin() &&
                    curPkt->tickSched < i->tickSched) {
                    // inserts the element before the position pointed
                    // to by the iterator, so before the beginning
                    xbar.pktToTransmitEBList.insert(i, *curPkt);
                } else {
                    // inserts the element before the position pointed
                    // to by the iterator, so advance it one step
                    xbar.pktToTransmitEBList.insert(++i, *curPkt);
                }
            }
            // Reschedule the collisionCheckEvent at the next tx end
            xbar.reschedule(xbar.collisionCheckEvent, 
                            xbar.pktToTransmitEBList.begin()->tickSched, true);

            // The timing fails for letting wireless to detect collisions

            return false;
        }
        case Enums::token_pass:
        {
            if (!xbar.tokenChangeEvent.scheduled()) {
                // Schedule first token change in case first cycle is silent
                xbar.reschedule(xbar.tokenChangeEvent, xbar.nextCycle(), true);
            }

            // @NOTE Immediate transmission if defaultPort
            if (src_id == xbar.getDefaultPortID() && layerType == Enums::RESP) {
                return true;
            }

            // Transmit if channel unnoccupied and holding the token
            if (xbar.channelStateIdle() && xbar.getTokenID() == src_id) {

                // Signal wireless channel as occupied
                xbar.channelStateToBusy();

                // Compute transmission length. If no payload, assume 1B
                Tick txLength = pkt->hasData() ? 
                        pkt->getSize() * xbar.bandwidth : xbar.bandwidth;

                // Register  stats
                occupancy += txLength;
                xbar.wirelessLatency[pkt->req->masterId()] += txLength;
                xbar.wirelessTransmissions[pkt->req->masterId()]++;
                unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
                xbar.bytesTransmitted[pkt->req->masterId()] += pkt_size;

                // Schedule tokenChangeEvent to turn channel back to idle 
                // and advance token ID after the end of the transmission
                xbar.reschedule(xbar.tokenChangeEvent, 
                        xbar.clockEdge(xbar.ticksToCycles(txLength)), true);

                return true;

            } else {    // Channel has to wait until it holds the token

                // The port should not be already in the list
                assert (std::find(waitingForLayer.begin(),
                    waitingForLayer.end(), src_port) == waitingForLayer.end());

                // Signal that port is waiting for its turn
                waitingForLayer.push_back(src_port);

                // Store layer and port data for sending the retry, and start
                // tick for computing latency
                TokenPassPktPtr curPkt = new TokenPassPkt(curTick(), layerType,
                        src_port, src_id, dest_port, pkt->req->masterId());

                xbar.pktToTransmitTPList.push_back(*curPkt);

                return false;
            }
        }
        default:
            // @NOTE empty for the moment
            return true;
    }
}

template <typename SrcType, typename DstType>
void
WirelessXBar::WirelessLayer<SrcType,DstType>::occupyLayer(Tick until)
{
    // ensure the state is busy at this point, as the layer should
    // transition from idle as soon as it has decided to forward the
    // packet to prevent any follow-on calls to sendTiming seeing an
    // unoccupied layer
    assert(state == BUSY);

    // until should never be 0 as express snoops never occupy the layer
    assert(until != 0);
    xbar.reschedule(releaseEvent, until, true);
}

template <typename SrcType, typename DstType>
void
WirelessXBar::WirelessLayer<SrcType,DstType>::succeededTiming(Tick busy_time)
{
    // we should have gone from idle or retry to busy in the tryTiming
    // test
    assert(state == BUSY);

    // occupy the layer accordingly
    occupyLayer(busy_time);
}

template <typename SrcType, typename DstType>
void
WirelessXBar::WirelessLayer<SrcType,DstType>::failedTiming(SrcType* src_port,
                                              Tick busy_time)
{
    // if the source port is the current retrying one or not, we have
    // failed in forwarding and should track that we are now waiting
    // for the peer to send a retry
    waitingForPeer.push_back(src_port);

    // we should have gone from idle or retry to busy in the tryTiming
    // test
    assert(state == BUSY);

    // occupy the bus accordingly
    occupyLayer(busy_time);
}

template <typename SrcType, typename DstType>
void
WirelessXBar::WirelessLayer<SrcType,DstType>::releaseLayer()
{
    // releasing the bus means we should be idle if no current transmissions
    assert(state == BUSY);

    switch (macProto) {
        case Enums::exp_backoff:
            if (waitingForLayer.empty()) {
                // update the state
                state = IDLE;
            }
            // We don't call retryWaiting from here, as waitingForLayer
            // are resolved through expBackoffCollisionCheck()
        break;
        case Enums::token_pass:
            state = IDLE;
            // We don't call retryWaiting from here as waitingForLayer are
            // resolved/updated every time the token changes
        break;
        default:
            // @NOTE empty for the moment
        break;
    }

    if (waitingForLayer.empty() && waitingForPeer.empty() &&
        drainState() == DrainState::Draining) {
        DPRINTF(Drain, "Crossbar done draining, signaling drain manager\n");
        //If we weren't able to drain before, do it now.
        signalDrainDone();
    }
}

template <typename SrcType, typename DstType>
void
WirelessXBar::WirelessLayer<SrcType,DstType>::retryWaiting()
{
    switch (macProto) {
        case Enums::exp_backoff:
            // Nothing here, as rescheduling and sendRetry are
            // managed from expBackoffCollisionCheck()
        break;
        case Enums::token_pass:
            // Nothing here, as rescheduling and sendRetry are
            // managed from tokenChange()
        break;
        default:
            // @NOTE empty for the moment
        break;
    }
}

template <typename SrcType, typename DstType>
void
WirelessXBar::WirelessLayer<SrcType,DstType>::recvRetry()
{
    // we don't mark the retries, as they will have to go again
    // through the collision process
    // we should never get a retry without having failed to forward
    // something to this port
    assert(!waitingForPeer.empty());

    switch (macProto) {
        case Enums::exp_backoff:
        case Enums::token_pass:
        {
            // Send a retry request to all the ports waiting 
            // @NOTE: No retrying problem as pkts in waitingForPeer aren't in
            // waitingForLayer, but maybe we're burdening the channel
            auto i = waitingForPeer.begin();
            while (i != waitingForPeer.end()) {
                SrcType* retryingPort = *i;
                i = waitingForPeer.erase(i);
                sendRetry(retryingPort);
            }
        }
        break;
        default:
            // @NOTE empty for the moment
        break;
    }
}

template <typename SrcType, typename DstType> 
void
WirelessXBar::WirelessLayer<SrcType,DstType>::triggerSendRetry(Port* srcPort)
{
    auto i = waitingForLayer.begin();
    while (*i != srcPort && i != waitingForLayer.end()) {
        ++i;
    }

    panic_if(i == waitingForLayer.end(),
                "Packet to be retried is not found in the list\n");

    SrcType* retryPort = (SrcType*) *i;

    switch (macProto) {
        case Enums::exp_backoff:
        {
            // @NOTE: packets are erased from the queues at tryTiming, 
            // to signal that the tx has not had any collisions
        }
        break;
        case Enums::token_pass:
        {
            // We erase packets from the queues here
            i = waitingForLayer.erase(i);

            // Find the packet in the pktToTransmitTPList
            auto j = xbar.pktToTransmitTPList.begin();
            while (j->srcPort != srcPort && j != xbar.pktToTransmitTPList.end())
                ++j;
            
            panic_if(j == xbar.pktToTransmitTPList.end(),
                "Packet to be transmitted is not found in the list\n");

            // Register latency for waiting for token (tx recorded at tryTiming)
            xbar.wirelessLatency[j->masterId] += curTick() - j->tickStart;
            if (curTick() - j->tickStart >
                                    xbar.cyclesToTicks(Cycles(xbar.numTokens)))
                xbar.wirelessCollisions[j->masterId]++;

            // Remove from pktToTransmitTPList
            j = xbar.pktToTransmitTPList.erase(j);
        }
        break;
        default:
            // @NOTE empty for the moment
        break;
    }
    sendRetry(retryPort);
}

template <typename SrcType, typename DstType>
DrainState
WirelessXBar::WirelessLayer<SrcType,DstType>::drain()
{
    //We should check that we're not "doing" anything, and that noone is
    //waiting. We might be idle but have someone waiting if the device we
    //contacted for a retry didn't actually retry.
    if (state != IDLE) {
        DPRINTF(Drain, "Crossbar not drained\n");
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

template <typename SrcType, typename DstType>
void
WirelessXBar::WirelessLayer<SrcType,DstType>::regStats()
{
    using namespace Stats;

    occupancy
        .name(name() + ".occupancy")
        .desc("Layer occupancy (ticks)")
        .flags(nozero);

    utilization
        .name(name() + ".utilization")
        .desc("Layer utilization (%)")
        .precision(1)
        .flags(nozero);

    utilization = 100 * occupancy / simTicks;
}

/**
 * Crossbar layer template instantiations. Could be removed with _impl.hh
 * file, but since there are only two given options (MasterPort and
 * SlavePort) it seems a bit excessive at this point.
 */
template class WirelessXBar::WirelessLayer<SlavePort,MasterPort>;
template class WirelessXBar::WirelessLayer<MasterPort,SlavePort>;