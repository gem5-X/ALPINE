#
# Copyright (c) 2021 EPFL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Rafael Medina

from MemObject import MemObject
from System import System
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from XBar import *

# Enum for MAC protocols
class MacProtocol(Enum): vals = ['exp_backoff', 'token_pass']

class LayerType(Enum): vals = ['REQ', 'RESP', 'SNOOP']

class WirelessXBar(BaseXBar):
    type = 'WirelessXBar'
    cxx_header = "mem/wireless_xbar.hh"

    bandwidth = Param.MemoryBandwidth('1.25GB/s', "Modeled write bandwidth")
    mac_protocol = Param.MacProtocol('exp_backoff', "MAC protocol")
    retry_slot_size = Param.Unsigned('64', "Slot size for retry windows")
    backoff_ceil = Param.Unsigned('10',"Max. exponent for exponential backoff")

    # The coherent crossbar additionally has snoop responses that are
    # forwarded after a specific latency.
    snoop_response_latency = Param.Cycles("Snoop response latency")

    # An optional snoop filter
    snoop_filter = Param.SnoopFilter(NULL, "Selected snoop filter")

    # Determine how this crossbar handles packets where caches have
    # already committed to responding, by establishing if the crossbar
    # is the point of coherency or not.
    point_of_coherency = Param.Bool(False, "Consider this crossbar the " \
                                    "point of coherency")

    # Specify whether this crossbar is the point of unification.
    point_of_unification = Param.Bool(False, "Consider this crossbar the " \
                                      "point of unification")

    system = Param.System(Parent.any, "System that the crossbar belongs to.")

# We can use a wireless crossbar to connect multiple masters to the L2
# caches.
class WirelessL2XBar(WirelessXBar):
    # 256-bit crossbar by default
    width = 32

    # Assume that most of this is covered by the cache latencies, with
    # no more than a single pipeline stage for any packet.
    frontend_latency = 1
    forward_latency = 0
    response_latency = 1
    snoop_response_latency = 1

    # Use a snoop-filter by default, and set the latency to zero as
    # the lookup is assumed to overlap with the frontend latency of
    # the crossbar
    snoop_filter = SnoopFilter(lookup_latency = 0)

    # This specialisation of the coherent crossbar is to be considered
    # the point of unification, it connects the dcache and the icache
    # to the first level of unified cache.
    point_of_unification = True

# We can use a wireless system interconnect, tying together the CPU clusters,
# GPUs, and any I/O coherent masters, and DRAM controllers.
class WirelessSystemXBar(WirelessXBar):
    # 128-bit crossbar by default
    width = 16

    # A handful pipeline stages for each portion of the latency
    # contributions.
    frontend_latency = 3
    forward_latency = 4
    response_latency = 2
    snoop_response_latency = 4

    # Use a snoop-filter by default
    snoop_filter = SnoopFilter(lookup_latency = 1)

    # This specialisation of the coherent crossbar is to be considered
    # the point of coherency, as there are no (coherent) downstream
    # caches.
    point_of_coherency = True

    # This specialisation of the coherent crossbar is to be considered
    # the point of unification, it connects the dcache and the icache
    # to the first level of unified cache. This is needed for systems
    # without caches where the SystemXBar is also the point of
    # unification.
    point_of_unification = True
    