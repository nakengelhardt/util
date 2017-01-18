from migen import *
from migen.genlib.record import *
from migen.genlib.roundrobin import *

import random
import logging

def pack(words):
    data = 0
    for i, word in enumerate(words):
        data = data | ((word & 0xFFFFFFFF) << i*32)
    return data

def unpack(data, n):
    words = []
    for i in range(n):
        words.append((data >> i*32) & 0xFFFFFFFF)
    return words

_stream_layout = [
    ("valid", 1, DIR_M_TO_S),
    ("rdy", 1, DIR_S_TO_M),
    ("data", "data_width", DIR_M_TO_S)
]

class PicoStreamInterface(Record):
    def __init__(self, data_width=128):
        Record.__init__(self, set_layout_parameters(_stream_layout, data_width=data_width))

    def write(channel, words):
        wordsize = 32
        channelwidth = len(channel.data)//wordsize
        nwords = len(words)
        nsent = 0
        while nsent < nwords:
            data = pack(words[nsent:min(nsent+channelwidth, nwords)])
            yield channel.data.eq(data)
            yield channel.valid.eq(1)
            yield
            while not (yield channel.rdy):
                yield
            nsent += channelwidth
        yield channel.valid.eq(0)
        yield

    def read(channel, nwords):
        wordsize = 32
        channelwidth = len(channel.data)//wordsize
        words = []
        yield channel.rdy.eq(1)
        while len(words) < nwords:
            if (yield channel.valid):
                data = (yield channel.data)
                words.extend(unpack(data, min(channelwidth, nwords-len(words))))
                if len(words) >= nwords:
                    yield channel.rdy.eq(0)
            yield
        print(words)

_bus_layout =[
    ("PicoAddr", "data_width", DIR_M_TO_S),
    ("PicoDataIn", "data_width", DIR_M_TO_S),
    ("PicoRd", 1, DIR_M_TO_S),
    ("PicoWr", 1, DIR_M_TO_S),
    ("PicoDataOut", "data_width", DIR_S_TO_M)
]

class PicoBusInterface(Record):
    def __init__(self, data_width=128):
        Record.__init__(self, set_layout_parameters(_bus_layout, data_width=data_width))

_hmc_port_layout = [
    ("clk", 1, DIR_M_TO_S),
    ("cmd_valid", 1, DIR_M_TO_S),
    ("cmd_ready", 1, DIR_S_TO_M),
    ("cmd", 4, DIR_M_TO_S),
    ("addr", "addr_width", DIR_M_TO_S),
    ("size", "size_width", DIR_M_TO_S),
    ("tag", 6, DIR_M_TO_S),
    ("wr_data", "data_width", DIR_M_TO_S),
    ("wr_data_valid", 1, DIR_M_TO_S),
    ("wr_data_ready", 1, DIR_S_TO_M),
    ("rd_data", "data_width", DIR_S_TO_M),
    ("rd_data_tag", 6, DIR_S_TO_M),
    ("rd_data_valid", 1, DIR_S_TO_M),
    ("errstat", 7, DIR_S_TO_M),
    ("dinv", 1, DIR_S_TO_M)
]

class HMCPort(Record):
    def __init__(self, **kwargs):
        Record.__init__(self, set_layout_parameters(_hmc_port_layout, **kwargs))
        self.effective_max_tag_size = 6

    @passive
    def gen_responses(self, data):
        logger = logging.getLogger('simulation.pico')
        # logger.setLevel(logging.INFO)
        logger.debug("start HMC sim")
        inflight = []
        while True:
            if len(inflight) < 64:
                (yield self.cmd_ready.eq(1))
            else:
                (yield self.cmd_ready.eq(0))

            if inflight and random.choice([True, False]):
                tag, val = random.choice(inflight)
                inflight.remove((tag, val))
                (yield self.rd_data.eq(val))
                (yield self.rd_data_tag.eq(tag))
                (yield self.rd_data_valid.eq(1))
                logger.debug("HMC reply: tag={} data=0x{:X}".format(tag, val))
            else:
                (yield self.rd_data_valid.eq(0))

            yield

            if (yield self.cmd_ready) and (yield self.cmd_valid):
                addr = (yield self.addr)
                size = (yield self.size)
                tag = (yield self.tag)
                logger.debug("HMC request: tag={} addr=0x{:X}".format(tag, addr))
                assert addr % 16 == 0
                assert size == 1
                idx = addr//4
                val = pack(data[idx:idx+4])
                inflight.append((tag, val))

class HMCPortMultiplexer(Module):
    def __init__(self, out_port, in_ports):
        if len(in_ports) == 0:
            return
        if len(in_ports) == 1:
            self.comb += in_ports[0].connect(out_port)
            return

        ## serve requests in roundrobin fashion

        # each port gets a subset of tags to use
        mux_bits = bits_for(len(in_ports)-1)
        effective_max_tag_size = out_port.effective_max_tag_size - mux_bits
        for port in in_ports:
            port.effective_max_tag_size = effective_max_tag_size

            # ("cmd_valid", 1, DIR_M_TO_S),
            # ("cmd_ready", 1, DIR_S_TO_M),
            # ("cmd", 4, DIR_M_TO_S),
            # ("addr", "addr_width", DIR_M_TO_S),
            # ("size", "size_width", DIR_M_TO_S),
            # ("tag", 6, DIR_M_TO_S),

        array_cmd_valid = Array(port.cmd_valid for port in in_ports)
        array_cmd_ready = Array(port.cmd_ready for port in in_ports)
        array_cmd = Array(port.cmd for port in in_ports)
        array_addr = Array(port.addr for port in in_ports)
        array_size = Array(port.size for port in in_ports)
        array_tag = Array(port.tag for port in in_ports)

        self.submodules.roundrobin = RoundRobin(len(in_ports), switch_policy=SP_CE)

        self.comb += [
            [self.roundrobin.request[i].eq(port.cmd_valid) for i, port in enumerate(in_ports)],
            self.roundrobin.ce.eq(out_port.cmd_ready),
            out_port.cmd_valid.eq(array_cmd_valid[self.roundrobin.grant]),
            array_cmd_ready[self.roundrobin.grant].eq(out_port.cmd_ready),
            out_port.cmd.eq(array_cmd[self.roundrobin.grant]),
            out_port.addr.eq(array_addr[self.roundrobin.grant]),
            out_port.size.eq(array_size[self.roundrobin.grant]),
            out_port.tag.eq(Cat(array_tag[self.roundrobin.grant][0:effective_max_tag_size], self.roundrobin.grant))
        ]

        ## distribute responses

        # ("rd_data", "data_width", DIR_S_TO_M),
        # ("rd_data_tag", 6, DIR_S_TO_M),
        # ("rd_data_valid", 1, DIR_S_TO_M),
        # ("dinv", 1, DIR_S_TO_M)

        array_data_valid = Array(port.rd_data_valid for port in in_ports)

        self.comb += [
            [port.rd_data.eq(out_port.rd_data) for port in in_ports],
            [port.rd_data_tag.eq(out_port.rd_data_tag[:effective_max_tag_size]) for port in in_ports],
            array_data_valid[out_port.rd_data_tag[effective_max_tag_size:]].eq(out_port.rd_data_valid),
            [port.dinv.eq(out_port.dinv) for port in in_ports]
        ]

class PicoPlatform(Module):
    def __init__(self, num_hmc_ports_required, bus_width=32, stream_width=128, hmc_addr_width=34, hmc_size_width=4, hmc_data_width=128):
        self.ios = set()
        self.bus = PicoBusInterface(data_width=bus_width)
        for name in [x[0] for x in _bus_layout]:
            getattr(self.bus, name).name_override = name
        self.ios |= {getattr(self.bus, name) for name in [x[0] for x in _bus_layout]}
        self.bus_clk = Signal(name_override="PicoClk")
        self.bus_rst = Signal(name_override="PicoRst")
        self.ios |= {self.bus_clk, self.bus_rst}
        self.streams = []
        self.stream_width = stream_width
        self.hmc_addr_width = hmc_addr_width
        self.hmc_size_width = hmc_size_width
        self.hmc_data_width = hmc_data_width
        if num_hmc_ports_required > 0:
            self.hmc_clk_rx = Signal(name_override="hmc_rx_clk")
            self.hmc_clk_tx = Signal(name_override="hmc_tx_clk")
            self.hmc_rst = Signal(name_override="hmc_rst")
            self.hmc_trained = Signal(name_override="hmc_trained")
            self.ios |= {self.hmc_clk_rx, self.hmc_clk_tx, self.hmc_rst, self.hmc_trained}
            self.picoHMCports = []
            for i in range(9):
                port = HMCPort(addr_width=self.hmc_addr_width, size_width=self.hmc_size_width, data_width=self.hmc_data_width)
                for name in [x[0] for x in _hmc_port_layout]:
                    getattr(port, name).name_override = "hmc_{}_p{}".format(name, i)
                self.ios |= {getattr(port, name) for name in [x[0] for x in _hmc_port_layout]}
                self.picoHMCports.append(port)
            self.HMCports = [HMCPort(addr_width=self.hmc_addr_width, size_width=self.hmc_size_width, data_width=self.hmc_data_width) for _ in range(num_hmc_ports_required)]
            portgroups = [list() for _ in range(9)]
            for i,port in enumerate(self.HMCports):
                portgroups[i % 9].append(port)
            for i in range(9):
                self.submodules += HMCPortMultiplexer(out_port=self.picoHMCports[i], in_ports=portgroups[i])

    def getExtraClk(self):
        if not hasattr(self, "extra_clk"):
            self.extra_clk = Signal(name_override="extra_clk")
            self.ios |= {self.extra_clk}
        return self.extra_clk

    def getBus(self):
        return self.bus

    def getBusClkRst(self):
        return self.bus_clk, self.bus_rst

    def ensureStreamClk(self):
        if not hasattr(self, "stream_clk"):
            self.stream_clk = Signal(name_override="clk")
            self.stream_rst = Signal(name_override="rst")
            self.ios |= {self.stream_clk, self.stream_rst}

    def getStreamPair(self):
        self.ensureStreamClk()
        rx = PicoStreamInterface(data_width=self.stream_width)
        tx = PicoStreamInterface(data_width=self.stream_width)
        self.streams.append((rx, tx))
        i = len(self.streams)
        for name in [x[0] for x in _stream_layout]:
                getattr(rx, name).name_override="s{}i_{}".format(i, name)
                getattr(tx, name).name_override="s{}o_{}".format(i, name)
        self.ios |= {getattr(rx, name) for name in [x[0] for x in _stream_layout]}
        self.ios |= {getattr(tx, name) for name in [x[0] for x in _stream_layout]}

        return rx, tx

    def getStreamClkRst(self):
        self.ensureStreamClk()
        return self.stream_clk, self.stream_rst

    def getHMCPort(self, i):
        return self.HMCports[i]

    def getHMCClkEtc(self):
        return self.hmc_clk_rx, self.hmc_clk_tx, self.hmc_rst, self.hmc_trained

    def get_ios(self):
        return self.ios

    def getSimGenerators(self, data):
        return [port.gen_responses(data) for port in self.picoHMCports]
