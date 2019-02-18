from migen import *
from migen.genlib.record import *
from migen.genlib.roundrobin import *
from migen.genlib.fifo import *
from migen.fhdl import verilog
import migen.build.xilinx.common

import random
import logging
import struct

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

def repack(data, n):
    packed_data = []
    for i in range(0, len(data), n):
        packed_data.append(pack(data[i:min(i+n, len(data))]))
    return packed_data

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
                data = unpack((yield channel.data), min(channelwidth, nwords-len(words)))
                for i, word in enumerate(reversed(data)):
                    print("{:08x}".format(word), end='\n' if i % 4 == 3 else '_')
                words.extend(data)
                if len(words) >= nwords:
                    yield channel.rdy.eq(0)
            yield
        return words

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

    def read(addr):
        yield self.PicoRd.eq(1)
        yield self.PicoWr.eq(0)
        yield self.PicoAddr.eq(addr)
        yield
        yield self.PicoRd.eq(0)
        return (yield self.PicoDataOut)

    def write(addr, data):
        yield self.PicoRd.eq(0)
        yield self.PicoWr.eq(1)
        yield self.PicoAddr.eq(addr)
        yield self.PicoDataIn.eq(data)
        yield
        yield self.PicoWr.eq(0)

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
    HMC_CMD_RD = 0b0000 # read
    HMC_CMD_WR = 0b1001 # posted write
    HMC_CMD_WR_NP = 0b0001 # non-posted write

    def __init__(self, **kwargs):
        Record.__init__(self, set_layout_parameters(_hmc_port_layout, **kwargs))
        self.effective_max_tag_size = 6
        self.hmc_data = []

    @passive
    def gen_responses(self):

        logger = logging.getLogger('simulation.pico')
        logger.setLevel(logging.INFO)
        logger.debug("start HMC sim")
        num_cycles = 0
        inflight = []
        cmd_inflight_w = []
        data_inflight_w = []
        while True:
            if random.choice([True, False]):
                (yield self.cmd_ready.eq(1))
            else:
                (yield self.cmd_ready.eq(0))

            if len(cmd_inflight_w) < len(data_inflight_w):
                (yield self.wr_data_ready.eq(0))
            else:
                (yield self.wr_data_ready.eq(1))

            case = random.choice([0,1,2])

            if case == 0 and cmd_inflight_w:
                i = random.randrange(len(cmd_inflight_w))
                if i < len(data_inflight_w):
                    tag, idx = cmd_inflight_w[i]
                    del cmd_inflight_w[i]
                    val = data_inflight_w[i]
                    del data_inflight_w[i]
                    if idx >= len(self.hmc_data):
                        self.hmc_data.extend(0 for _ in range(len(self.hmc_data), idx+1))
                    self.hmc_data[idx] = val
                    if tag >= 0:
                        (yield self.rd_data_tag.eq(tag))
                        (yield self.rd_data_valid.eq(1))
                    else:
                        (yield self.rd_data_valid.eq(0))
                else:
                    (yield self.rd_data_valid.eq(0))
            elif case == 1 and inflight:
                tag, idx = random.choice(inflight)
                val = self.hmc_data[idx]
                inflight.remove((tag, idx))
                (yield self.rd_data.eq(val))
                (yield self.rd_data_tag.eq(tag))
                (yield self.rd_data_valid.eq(1))
                logger.debug("{}: HMC reply: tag={} data=0x{:X}".format(num_cycles, tag, val))
            else:
                (yield self.rd_data_valid.eq(0))


            yield
            num_cycles += 1

            if (yield self.cmd_ready) and (yield self.cmd_valid):
                addr = (yield self.addr)
                size = (yield self.size)
                tag = (yield self.tag)
                cmd = (yield self.cmd)
                logger.debug("{}: HMC request: {} tag={} addr=0x{:X}".format(num_cycles, "read" if cmd == self.HMC_CMD_RD else "write", tag, addr))
                assert addr % 16 == 0
                assert size == 1
                idx = addr >> 4

                if cmd == self.HMC_CMD_RD:
                    inflight.append((tag, idx))
                elif cmd == self.HMC_CMD_WR:
                    cmd_inflight_w.append((-1, idx))
                elif cmd == self.HMC_CMD_WR_NP:
                    cmd_inflight_w.append((tag, idx))
                else:
                    raise NotImplementedError

            if (yield self.wr_data_ready) and (yield self.wr_data_valid):
                val = (yield self.wr_data)
                data_inflight_w.append(val)

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

        ## write requests not implemented

        self.comb += [
            out_port.wr_data.eq(0),
            out_port.wr_data_valid.eq(0)
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

class HMCPortWriteUnifier(Module):
    def __init__(self, pico_port):
        usr_port_layout = [
            ("cmd_valid", 1),
            ("cmd_ready", 1),
            ("cmd", 4),
            ("addr", len(pico_port.addr)),
            ("size", len(pico_port.size)),
            ("tag", 6),
            ("wr_data", len(pico_port.wr_data)),
            ("rd_data", len(pico_port.rd_data)),
            ("rd_data_tag", 6),
            ("rd_data_valid", 1)
        ]
        for name, length in usr_port_layout:
            setattr(self, name, Signal(length, name_override=name))
        self.effective_max_tag_size = pico_port.effective_max_tag_size

        cmd_layout = [
            ("cmd", len(pico_port.cmd)),
            ("addr", len(pico_port.addr)),
            ("size", len(pico_port.size)),
            ("tag", len(pico_port.tag))
        ]
        self.submodules.cmd_fifo = SyncFIFO(width=layout_len(cmd_layout), depth=8)
        self.submodules.data_fifo = SyncFIFO(width=len(pico_port.wr_data), depth=8)
        cmd_fifo_in = Record(cmd_layout)
        cmd_fifo_out = Record(cmd_layout)

        self.comb += [
            # cmd fifo split din/dout into fields
            self.cmd_fifo.din.eq(cmd_fifo_in.raw_bits()),
            cmd_fifo_out.raw_bits().eq(self.cmd_fifo.dout),
            # inputs
            cmd_fifo_in.cmd.eq(self.cmd),
            cmd_fifo_in.addr.eq(self.addr),
            cmd_fifo_in.size.eq(self.size),
            cmd_fifo_in.tag.eq(self.tag),
            self.data_fifo.din.eq(self.wr_data),
            # input flow control
            self.cmd_ready.eq(self.cmd_fifo.writable & self.data_fifo.writable),
            self.cmd_fifo.we.eq(self.cmd_ready & self.cmd_valid),
            self.data_fifo.we.eq(self.cmd_ready & self.cmd_valid & (self.cmd != pico_port.HMC_CMD_RD)),
            # output cmd
            pico_port.cmd.eq(cmd_fifo_out.cmd),
            pico_port.addr.eq(cmd_fifo_out.addr),
            pico_port.size.eq(cmd_fifo_out.size),
            pico_port.tag.eq(cmd_fifo_out.tag),
            pico_port.cmd_valid.eq(self.cmd_fifo.readable),
            self.cmd_fifo.re.eq(pico_port.cmd_ready),
            # output data
            pico_port.wr_data.eq(self.data_fifo.dout),
            pico_port.wr_data_valid.eq(self.data_fifo.readable),
            self.data_fifo.re.eq(pico_port.wr_data_ready),
            # connect read data
            self.rd_data.eq(pico_port.rd_data),
            self.rd_data_tag.eq(pico_port.rd_data_tag),
            self.rd_data_valid.eq(pico_port.rd_data_valid)
        ]

class PicoPlatform(Module):
    def __init__(self, num_hmc_ports_required, bus_width=32, stream_width=128, hmc_addr_width=34, hmc_size_width=4, hmc_data_width=128, init=None):
        self.ios = set()
        self.streams = []
        self.stream_width = stream_width
        self.bus_width = bus_width
        self.hmc_addr_width = hmc_addr_width
        self.hmc_size_width = hmc_size_width
        self.hmc_data_width = hmc_data_width
        if init:
            self.init_data = init
        if num_hmc_ports_required > 0:
            self.makeHMCports(num_hmc_ports_required)
            self.hmc_data = repack(init, 4) if init else []
            for port in self.picoHMCports:
                port.hmc_data = self.hmc_data

    def makeHMCports(self, num_hmc_ports_required):
        self.clock_domains.cd_sys = ClockDomain()
        self.hmc_clk_rx = Signal(name_override="hmc_rx_clk")
        self.cd_sys.clk.name_override = "hmc_tx_clk"
        self.cd_sys.rst.name_override = "hmc_rst"
        self.hmc_trained = Signal(name_override="hmc_trained", reset=1)
        self.ios |= {self.hmc_clk_rx, self.cd_sys.clk, self.cd_sys.rst, self.hmc_trained}
        self.picoHMCports = []
        for i in range(9):
            port = HMCPort(addr_width=self.hmc_addr_width, size_width=self.hmc_size_width, data_width=self.hmc_data_width)
            for name in [x[0] for x in _hmc_port_layout]:
                getattr(port, name).name_override = "hmc_{}_p{}".format(name, i)
            self.ios |= {getattr(port, name) for name in [x[0] for x in _hmc_port_layout]}
            self.picoHMCports.append(port)
            self.comb += port.clk.eq(self.cd_sys.clk)
            if i >= num_hmc_ports_required:
                for field, _, dir in _hmc_port_layout:
                    if field != "clk" and dir == DIR_M_TO_S:
                        s = getattr(port, field)
                        self.comb += s.eq(0)

        if num_hmc_ports_required <= 9:
            self.HMCports = self.picoHMCports
        else:
            # this is broken
            raise NotImplementedError
            self.HMCports = [HMCPort(addr_width=self.hmc_addr_width, size_width=self.hmc_size_width, data_width=self.hmc_data_width) for _ in range(num_hmc_ports_required)]
            portgroups = [list() for _ in range(9)]
            for i,port in enumerate(self.HMCports):
                portgroups[i % 9].append(port)
            for i in range(9):
                self.submodules += HMCPortMultiplexer(out_port=self.picoHMCports[i], in_ports=portgroups[i])


    def ensureStreamClk(self):
        if not hasattr(self, "cd_stream"):
            self.clock_domains.cd_stream = ClockDomain()
            self.cd_stream.clk.name_override = "clk"
            self.cd_stream.rst.name_override = "rst"
            self.ios |= {self.cd_stream.clk, self.cd_stream.rst}

    def ensureBus(self):
        if not hasattr(self, "cd_bus"):
            self.bus = PicoBusInterface(data_width=self.bus_width)
            for name in [x[0] for x in _bus_layout]:
                getattr(self.bus, name).name_override = name
            self.ios |= {getattr(self.bus, name) for name in [x[0] for x in _bus_layout]}
            self.clock_domains.cd_bus = ClockDomain()
            self.cd_bus.clk.name_override = "PicoClk"
            self.cd_bus.rst.name_override = "PicoRst"
            self.ios |= {self.cd_bus.clk, self.cd_bus.rst}

    def getExtraClk(self):
        if not hasattr(self, "extra_clk"):
            self.clock_domains.cd_extra = ClockDomain(reset_less=True)
            self.cd_extra.clk.name_override = "extra_clk"
            self.ios |= {self.cd_extra.clk}
        return self.cd_extra

    def getBus(self):
        self.ensureBus()
        return self.bus

    def getBusClk(self):
        self.ensureBus()
        return self.cd_bus

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
        return self.cd_stream

    def getHMCPort(self, i):
        return self.HMCports[i]

    def getHMCClkEtc(self):
        if not hasattr(self, "cd_sys"):
            self.makeHMCports(0)
        return self.hmc_clk_rx, self.cd_sys.clk, self.cd_sys.rst, self.hmc_trained

    def get_ios(self):
        return self.ios

    def getSimGenerators(self):
        if hasattr(self, "cd_sys"):
            generators = {"sys": [port.gen_responses() for port in self.picoHMCports]}
        else:
            generators = dict()
        return generators

    def simulate(self, user_module, user_cd="sys", vcd_name="tb.vcd"):
        self.submodules.user_module = user_module
        generators = self.getSimGenerators()
        generators[user_cd].extend(get_simulators(user_module, 'gen_selfcheck', user_module))
        generators[user_cd].extend(get_simulators(user_module, 'gen_simulation', user_module))
        run_simulation(self, generators, clocks={"sys": 10, "bus": 480, "stream": 8}, vcd_name=vcd_name)

    def export(self, user_module, module_name="top", filename="top.v"):
        self.submodules.user_module = user_module
        so = dict(migen.build.xilinx.common.xilinx_special_overrides)
        verilog.convert(self,
                        name=module_name,
                        ios=self.get_ios(),
                        special_overrides=so,
                        create_clock_domains=False
                        ).write(filename)
        if hasattr(self, "init_data"):
            with open("hmc_init.data", 'wb') as f:
                for x in self.init_data:
                    f.write(struct.pack('=I', x))
