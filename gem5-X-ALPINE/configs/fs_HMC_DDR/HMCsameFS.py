# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2015 The University of Bologna
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
# Authors: Erfan Azarkhish
#          Abdul Mutaal Ahmad

# A Simplified model of a complete HMC device. Based on:
#  [1] http://www.hybridmemorycube.org/specification-download/
#  [2] High performance AXI-4.0 based interconnect for extensible smart memory
#      cubes(E. Azarkhish et. al)
#  [3] Low-Power Hybrid Memory Cubes With Link Power Management and Two-Level
#      Prefetching (J. Ahn et. al)
#  [4] Memory-centric system interconnect design with Hybrid Memory Cubes
#      (G. Kim et. al)
#  [5] Near Data Processing, Are we there yet? (M. Gokhale)
#      http://www.cs.utah.edu/wondp/gokhale.pdf
#  [6] openHMC - A Configurable Open-Source Hybrid Memory Cube Controller
#      (J. Schmidt)
#  [7] Hybrid Memory Cube performance characterization on data-centric
#      workloads (M. Gokhale)
#
# This script builds a complete HMC device composed of vault controllers,
# serial links, the main internal crossbar, and an external hmc controller.
#
# - VAULT CONTROLLERS:
#   Instances of the HMC_2500_1x32 class with their functionality specified in
#   dram_ctrl.cc
#
# - THE MAIN XBAR:
#   This component is simply an instance of the NoncoherentXBar class, and its
#   parameters are tuned to [2].
#
# - SERIAL LINKS CONTROLLER:
#   SerialLink is a simple variation of the Bridge class, with the ability to
#   account for the latency of packet serialization and controller latency. We
#   assume that the serializer component at the transmitter side does not need
#   to receive the whole packet to start the serialization. But the
#   deserializer waits for the complete packet to check its integrity first.
#
#   * Bandwidth of the serial links is not modeled in the SerialLink component
#     itself.
#
#   * Latency of serial link controller is composed of SerDes latency + link
#     controller
#
#   * It is inferred from the standard [1] and the literature [3] that serial
#     links share the same address range and packets can travel over any of
#     them so a load distribution mechanism is required among them.
#
#   -----------------------------------------
#   | Host/HMC Controller                   |
#   |        ----------------------         |
#   |        |  Link Aggregator   |  opt    |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Serial Link + Ser | * 4     |
#   |        ----------------------         |
#   |---------------------------------------
#   -----------------------------------------
#   | Device
#   |        ----------------------         |
#   |        |       Xbar         | * 4     |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Vault Controller  | * 16    |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |     Memory         |         |
#   |        ----------------------         |
#   |---------------------------------------|
#
#   In this version we have present 3 different HMC archiecture along with
#   alongwith their corresponding test script.
#
#   same: It has 4 crossbars in HMC memory. All the crossbars are connected
#   to each other, providing complete memory range. This archicture also covers
#   the added latency for sending a request to non-local vault(bridge in b/t
#   crossbars). All the 4 serial links can access complete memory. So each
#   link can be connected to separate processor.
#
#   distributed: It has 4 crossbars inside the HMC. Crossbars are not
#   connected.Through each crossbar only local vaults can be accessed. But to
#   support this architecture we need a crossbar between serial links and
#   processor.
#
#   mixed: This is a hybrid architecture. It has 4 crossbars inside the HMC.
#   2 Crossbars are connected to only local vaults. From other 2 crossbar, a
#   request can be forwarded to any other vault.

import argparse

import m5
from m5.objects import *
from m5.util import *


def add_arguments(parser):
    # *****************************CROSSBAR PARAMETERS*************************
    # Flit size of the main interconnect [1]
    parser.add_argument("--xbar-width", default=32, action="store", type=int,
                        help="Data width of the main XBar (Bytes)")

    # Clock frequency of the main interconnect [1]
    # This crossbar, is placed on the logic-based of the HMC and it has its
    # own voltage and clock domains, different from the DRAM dies or from the
    # host.
    parser.add_argument("--xbar-frequency", default='1GHz', type=str,
                        help="Clock Frequency of the main XBar")

    # Arbitration latency of the HMC XBar [1]
    parser.add_argument("--xbar-frontend-latency", default=1, action="store",
                        type=int, help="Arbitration latency of the XBar")

    # Latency to forward a packet via the interconnect [1](two levels of FIFOs
    # at the input and output of the inteconnect)
    parser.add_argument("--xbar-forward-latency", default=2, action="store",
                        type=int, help="Forward latency of the XBar")

    # Latency to forward a response via the interconnect [1](two levels of
    # FIFOs at the input and output of the inteconnect)
    parser.add_argument("--xbar-response-latency", default=2, action="store",
                        type=int, help="Response latency of the XBar")

    # number of cross which connects 16 Vaults to serial link[7]
    parser.add_argument("--number-mem-crossbar", default=4, action="store",
                        type=int, help="Number of crossbar in HMC")

    # *****************************SERIAL LINK PARAMETERS**********************
    # Number of serial links controllers [1]
    parser.add_argument("--num-links-controllers", default=4, action="store",
                        type=int, help="Number of serial links")

    # Number of packets (not flits) to store at the request side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    parser.add_argument("--link-buffer-size-req", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        request side of the serial link")

    # Number of packets (not flits) to store at the response side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    parser.add_argument("--link-buffer-size-rsp", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        response side of the serial link")

    # Latency of the serial link composed by SER/DES latency (1.6ns [4]) plus
    # the PCB trace latency (3ns Estimated based on [5])
    parser.add_argument("--link-latency", default='4.6ns', type=str,
                        help="Latency of the serial links")

    # Clock frequency of the each serial link(SerDes) [1]
    parser.add_argument("--link-frequency", default='10GHz', type=str,
                        help="Clock Frequency of the serial links")

    # Clock frequency of serial link Controller[6]
    # clk_hmc[Mhz]= num_lanes_per_link * lane_speed [Gbits/s] /
    # data_path_width * 10^6
    # clk_hmc[Mhz]= 16 * 10 Gbps / 256 * 10^6 = 625 Mhz
    parser.add_argument("--link-controller-frequency", default='625MHz',
                        type=str, help="Clock Frequency of the link\
                        controller")

    # Latency of the serial link controller to process the packets[1][6]
    # (ClockDomain = 625 Mhz )
    # used here for calculations only
    parser.add_argument("--link-ctrl-latency", default=4, action="store",
                        type=int, help="The number of cycles required for the\
                        controller to process the packet")

    # total_ctrl_latency = link_ctrl_latency + link_latency
    # total_ctrl_latency = 4(Cycles) * 1.6 ns +  4.6 ns
    parser.add_argument("--total-ctrl-latency", default='30ns', type=str,
                        help="The latency experienced by every packet\
                        regardless of size of packet")

    # Number of parallel lanes in each serial link [1]
    parser.add_argument("--num-lanes-per-link", default=2, action="store",
                        type=int, help="Number of lanes per each link")

    # Number of serial links [1]
    parser.add_argument("--num-serial-links", default=1, action="store",
                        type=int, help="Number of serial links")

    # speed of each lane of serial link - SerDes serial interface 10 Gb/s
    parser.add_argument("--serial-link-speed", default=1, action="store",
                        type=int, help="Gbs/s speed of each lane of serial\
                        link")

    # address range for each of the serial links
    parser.add_argument("--serial-link-addr-range", default='4GB', type=str,
                        help="memory range for each of the serial links.\
                        Default: 4GB")

    # *****************************PERFORMANCE MONITORING*********************
    # The main monitor behind the HMC Controller
    parser.add_argument("--enable-global-monitor", action="store_true",
                        help="The main monitor behind the HMC Controller")

    # The link performance monitors
    parser.add_argument("--enable-link-monitor", action="store_true",
                        help="The link monitors")

    # link aggregator enable - put a cross between buffers & links
    parser.add_argument("--enable-link-aggr", action="store_true", help="The\
                        crossbar between port and Link Controller")

    parser.add_argument("--enable-buff-div", action="store_true",
                        help="Memory Range of Buffer is ivided between total\
                        range")

    # *****************************HMC ARCHITECTURE **************************
    # Memory chunk for 16 vault - numbers of vault / number of crossbars
    parser.add_argument("--mem-chunk", default=4, action="store", type=int,
                        help="Chunk of memory range for each cross bar in\
                        arch 0")

    # size of req buffer within crossbar, used for modelling extra latency
    # when the reuqest go to non-local vault
    parser.add_argument("--xbar-buffer-size-req", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        request side of the crossbar")

    # size of response buffer within crossbar, used for modelling extra latency
    # when the response received from non-local vault
    parser.add_argument("--xbar-buffer-size-resp", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        response side of the crossbar")
    # HMC device architecture. It affects the HMC host controller as well
    parser.add_argument("--arch", type=str, choices=["same", "distributed",
                        "mixed"], default="same", help="same: HMC with\
                        4 links, all with same range.\ndistributed: HMC with\
                        4 links with distributed range.\nmixed: mixed with\
                        same and distributed range.\nDefault: distributed")
    # HMC device - number of vaults
    parser.add_argument("--hmc-dev-num-vaults", default=16, action="store",
                        type=int, help="number of independent vaults within\
                        the HMC device. Note: each vault has a memory\
                        controller (valut controller)\nDefault: 16")
    # HMC device - vault capacity or size
    parser.add_argument("--hmc-dev-vault-size", default='256MB', type=str,
                        help="vault storage capacity in bytes. Default:\
                        256MB")
    parser.add_argument("--mem-type", type=str, choices=["HMC_2500_1x32"],
                        default="HMC_2500_1x32", help="type of HMC memory to\
                        use. Default: HMC_2500_1x32")
    parser.add_argument("--mem-channels", default=1, action="store", type=int,
                        help="Number of memory channels")
    parser.add_argument("--mem-ranks", default=1, action="store", type=int,
                        help="Number of ranks to iterate across")
    parser.add_argument("--burst-length", default=256, action="store",
                        type=int, help="burst length in bytes. Note: the\
                        cache line size will be set to this value.\nDefault:\
                        256")


def add_options(parser):
    # *****************************CROSSBAR PARAMETERS*************************
    # Flit size of the main interconnect [1]
    parser.add_option("--xbar-width", default=32, action="store", type=int,
                        help="Data width of the main XBar (Bytes)")

    # Clock frequency of the main interconnect [1]
    # This crossbar, is placed on the logic-based of the HMC and it has its
    # own voltage and clock domains, different from the DRAM dies or from the
    # host.
    parser.add_option("--xbar-frequency", default='1GHz', type=str,
                        help="Clock Frequency of the main XBar")

    # Arbitration latency of the HMC XBar [1]
    parser.add_option("--xbar-frontend-latency", default=1, action="store",
                        type=int, help="Arbitration latency of the XBar")

    # Latency to forward a packet via the interconnect [1](two levels of FIFOs
    # at the input and output of the inteconnect)
    parser.add_option("--xbar-forward-latency", default=2, action="store",
                        type=int, help="Forward latency of the XBar")

    # Latency to forward a response via the interconnect [1](two levels of
    # FIFOs at the input and output of the inteconnect)
    parser.add_option("--xbar-response-latency", default=2, action="store",
                        type=int, help="Response latency of the XBar")

    # number of cross which connects 16 Vaults to serial link[7]
    parser.add_option("--number-mem-crossbar", default=4, action="store",
                        type=int, help="Number of crossbar in HMC")

    # *****************************SERIAL LINK PARAMETERS**********************
    # Number of serial links controllers [1]
    parser.add_option("--num-links-controllers", default=4, action="store",
                        type=int, help="Number of serial links")

    # Number of packets (not flits) to store at the request side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    parser.add_option("--link-buffer-size-req", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        request side of the serial link")

    # Number of packets (not flits) to store at the response side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    parser.add_option("--link-buffer-size-rsp", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        response side of the serial link")

    # Latency of the serial link composed by SER/DES latency (1.6ns [4]) plus
    # the PCB trace latency (3ns Estimated based on [5])
    parser.add_option("--link-latency", default='4.6ns', type=str,
                        help="Latency of the serial links")

    # Clock frequency of the each serial link(SerDes) [1]
    parser.add_option("--link-frequency", default='10GHz', type=str,
                        help="Clock Frequency of the serial links")

    # Clock frequency of serial link Controller[6]
    # clk_hmc[Mhz]= num_lanes_per_link * lane_speed [Gbits/s] /
    # data_path_width * 10^6
    # clk_hmc[Mhz]= 16 * 10 Gbps / 256 * 10^6 = 625 Mhz
    parser.add_option("--link-controller-frequency", default='625MHz',
                        type=str, help="Clock Frequency of the link\
                        controller")

    # Latency of the serial link controller to process the packets[1][6]
    # (ClockDomain = 625 Mhz )
    # used here for calculations only
    parser.add_option("--link-ctrl-latency", default=4, action="store",
                        type=int, help="The number of cycles required for the\
                        controller to process the packet")

    # total_ctrl_latency = link_ctrl_latency + link_latency
    # total_ctrl_latency = 4(Cycles) * 1.6 ns +  4.6 ns
    parser.add_option("--total-ctrl-latency", default='30ns', type=str,
                        help="The latency experienced by every packet\
                        regardless of size of packet")

    # Number of parallel lanes in each serial link [1]
    parser.add_option("--num-lanes-per-link", default=2, action="store",
                        type=int, help="Number of lanes per each link")

    # Number of serial links [1]
    parser.add_option("--num-serial-links", default=1, action="store",
                        type=int, help="Number of serial links")

    # speed of each lane of serial link - SerDes serial interface 10 Gb/s
    parser.add_option("--serial-link-speed", default=10, action="store",
                        type=int, help="Gbs/s speed of each lane of serial\
                        link")

    # address range for each of the serial links
    parser.add_option("--serial-link-addr-range", default='4GB', type=str,
                        help="memory range for each of the serial links.\
                        Default: 4GB")

    # *****************************PERFORMANCE MONITORING*********************
    # The main monitor behind the HMC Controller
    parser.add_option("--enable-global-monitor", action="store_true",
                        help="The main monitor behind the HMC Controller")

    # The link performance monitors
    parser.add_option("--enable-link-monitor", action="store_true",
                        help="The link monitors")

    # link aggregator enable - put a cross between buffers & links
    parser.add_option("--enable-link-aggr", action="store_true", help="The\
                        crossbar between port and Link Controller")

    parser.add_option("--enable-buff-div", action="store_true",
                        help="Memory Range of Buffer is ivided between total\
                        range")

    # *****************************HMC ARCHITECTURE **************************
    # Memory chunk for 16 vault - numbers of vault / number of crossbars
    parser.add_option("--mem-chunk", default=4, action="store", type=int,
                        help="Chunk of memory range for each cross bar in\
                        arch 0")

    # size of req buffer within crossbar, used for modelling extra latency
    # when the reuqest go to non-local vault
    parser.add_option("--xbar-buffer-size-req", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        request side of the crossbar")

    # size of response buffer within crossbar, used for modelling extra latency
    # when the response received from non-local vault
    parser.add_option("--xbar-buffer-size-resp", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        response side of the crossbar")
    # HMC device architecture. It affects the HMC host controller as well
    parser.add_option("--arch", type=str, default="same", help="same: HMC with\
                        4 links, all with same range.\ndistributed: HMC with\
                        4 links with distributed range.\nmixed: mixed with\
                        same and distributed range.\nDefault: distributed")
    # HMC device - number of vaults
    parser.add_option("--hmc-dev-num-vaults", default=16, action="store",
                        type=int, help="number of independent vaults within\
                        the HMC device. Note: each vault has a memory\
                        controller (valut controller)\nDefault: 16")
    # HMC device - vault capacity or size
    parser.add_option("--hmc-dev-vault-size", default='256MB', type=str,
                        help="vault storage capacity in bytes. Default:\
                        256MB")
    parser.add_option("--burst-length", default=256, action="store",
                        type=int, help="burst length in bytes. Note: the\
                        cache line size will be set to this value.\nDefault:\
                        256")


# configure HMC host controller
def config_hmc_host_ctrl(opt, system, hmc_size):

    # create HMC host controller
    system.hmc_host = SubSystem()
    if (opt.no_ddr == False):
        start_slar =  system.mem_ranges[1].start # 0x3ffffffff
    else :
        start_slar =  system.mem_ranges[0].start # 0x3ffffffff

    # create memory ranges for the serial links
    #if(opt.num_serial_links > 0):
    #    slar = convert.toMemorySize(hmc_size) / opt.num_serial_links
    #else:
    slar = convert.toMemorySize(hmc_size)
    # Memmory ranges of serial link for arch-0. Same as the ranges of vault
    # controllers (4 vaults to 1 serial link)
    ser_ranges = [AddrRange(start_slar, start_slar+slar-1) for i in
                    range(opt.num_cpus)]


    # Serial link Controller with 16 SerDes links at 10 Gbps with serial link
    # ranges w.r.t to architecture
    sl = [Bridge(ranges=ser_ranges[i])
            for i in xrange(opt.alt_num_cpus)]
    print(len(sl))
    for i in xrange(opt.main_num_cpus):
        serlink = SerialLink(ranges=ser_ranges[i],
                    req_size=opt.link_buffer_size_req,
                    resp_size=opt.link_buffer_size_rsp,
                    num_lanes=opt.num_lanes_per_link,
                    link_speed=opt.serial_link_speed,
                    delay=opt.total_ctrl_latency)
        clk = opt.link_controller_frequency
        vd = VoltageDomain(voltage='1V')
        scd = SrcClockDomain(clock=clk, voltage_domain=vd)
        serlink.clk_domain = scd
        sl.append(serlink)
    print(len(sl))
    system.hmc_host.seriallink = sl

    # set the clock frequency for serial link
    """
    for i in xrange(opt.num_serial_links):
        clk = opt.link_controller_frequency
        vd = VoltageDomain(voltage='1V')
        scd = SrcClockDomain(clock=clk, voltage_domain=vd)
        system.hmc_host.seriallink[i].clk_domain = scd"""

    return system

def generate_addr_ranges(start, r_size, nbr, system):
    import math
    intlv_size = max(128, system.cache_line_size.value)
    intlv_low_bit = int(math.log(intlv_size, 2))
    intlv_bits =    int(math.log(nbr, 2))
    xor_low_bit = 20
    addr_ranges = []
    for i in xrange(0, nbr):
        range = m5.objects.AddrRange(start, size = r_size,
                                      intlvHighBit = \
                                          intlv_low_bit + intlv_bits - 1,
                                      xorHighBit = \
                                          xor_low_bit + intlv_bits - 1,
                                      intlvBits = intlv_bits,
                                      intlvMatch = i)
        addr_ranges.append(range)
    return addr_ranges



# Create an HMC device
def config_hmc_dev(opt, system, hmc_host):

    # create HMC device
    system.hmc_dev = SubSystem()
    if (opt.no_ddr == False):
        r = system.mem_ranges[1]
    else:
        r = system.mem_ranges[0]
    n = opt.hmc_dev_num_vaults
    # create memory ranges for the vault controllers
    #arv = convert.toMemorySize(opt.hmc_dev_vault_size)
    #addr_ranges_vaults = [AddrRange(start_adra + i*arv,
    #                    start_adra + ((i+1)*arv-1)) for i in
    #                    range(opt.hmc_dev_num_vaults)]
    bridges_ranges = generate_addr_ranges(r.start, r.size(), n, system)



    # 4 HMC Crossbars located in its logic-base (LoB)
    xb = [NoncoherentXBar(width=opt.xbar_width,
                          frontend_latency=opt.xbar_frontend_latency,
                          forward_latency=opt.xbar_forward_latency,
                          response_latency=opt.xbar_response_latency) for i in
          xrange(opt.number_mem_crossbar)]
    system.hmc_dev.xbar = xb

    for i in xrange(opt.number_mem_crossbar):
        clk = opt.xbar_frequency
        vd = VoltageDomain(voltage='1V')
        scd = SrcClockDomain(clock=clk, voltage_domain=vd)
        system.hmc_dev.xbar[i].clk_domain = scd

    # Attach serial link to 4 crossbar/s
    for i in xrange(opt.num_cpus):
        system.hmc_host.seriallink[i].master = \
            system.hmc_dev.xbar[i%opt.number_mem_crossbar].slave

    # Connecting xbar with each other for request arriving at the wrong xbar,
    # then it will be forward to correct xbar. Bridge is used to connect xbars

    numx = len(system.hmc_dev.xbar)

    # create a list of buffers
    if(numx > 1):
        system.hmc_dev.buffers = [Bridge(req_size=opt.xbar_buffer_size_req,
                                            resp_size=opt.xbar_buffer_size_resp)
                                    for i in xrange(numx*(numx-1))]

        # Buffer iterator
        it = iter(range(len(system.hmc_dev.buffers)))

        # iterate over all the crossbars and connect them as required
        import math
        vaults_per_xbar = int(math.ceil(
            float(opt.hmc_dev_num_vaults)/
            float(opt.number_mem_crossbar)))

        for i in range(numx):
            for j in range(numx):
                # connect xbar to all other xbars except itself
                if i != j:
                    # get the next index of buffer
                    index = it.next()

                    # Change the default values for ranges of bridge
                    system.hmc_dev.buffers[index].ranges = \
                        bridges_ranges[
                            j * int(vaults_per_xbar):
                            (j + 1) * int(vaults_per_xbar)
                        ]

                    # Connect the bridge between corssbars
                    system.hmc_dev.xbar[i].master = system.hmc_dev.buffers[
                            index].slave
                    system.hmc_dev.buffers[
                            index].master = system.hmc_dev.xbar[j].slave
                else:
                    # Don't connect the xbar to itself
                    pass
