from migen import *
from migen.genlib.fifo import *

from .pico import HMCPort

class HMCReorderBuffer(Module):
    def __init__(self, hmc_port):

        self.req = Record([
            ("addr", len(hmc_port.addr)),
            ("size", len(hmc_port.size)),
            ("valid", 1),
            ("ack", 1)
        ])
        self.rep = Record([
            ("data", len(hmc_port.rd_data)),
            ("valid", 1),
            ("ack", 1)
        ])

        tag_sz = hmc_port.effective_max_tag_size
        num_tags = 2**tag_sz

        self.specials.reorder_buffer = Memory(width=len(hmc_port.rd_data), depth=num_tags*8)
        self.specials.wr_port = wr_port = self.reorder_buffer.get_port(write_capable=True, mode=READ_FIRST)
        self.specials.rd_port = rd_port = self.reorder_buffer.get_port(async_read=True, mode=READ_FIRST)

        self.specials.size_buffer = Memory(width=len(hmc_port.size), depth=num_tags)
        size_wr_port = self.size_buffer.get_port(write_capable=True, mode=READ_FIRST)
        size_rd_port = self.size_buffer.get_port(async_read=True, mode=READ_FIRST)
        size_tag_port = self.size_buffer.get_port(async_read=True, mode=READ_FIRST)
        self.specials += size_wr_port
        self.specials += size_rd_port
        self.specials += size_tag_port

        tag_in_use = Array(Signal() for _ in range(num_tags))
        data_valid = Array(Signal() for _ in range(num_tags))

        tag_available = Signal()

        wr_ptr = Signal(tag_sz)
        rd_ptr = Signal(tag_sz)

        wr_flit = Signal(3)
        rd_flit = Signal(3)
        prev_tag = Signal(tag_sz)

        self.burst_order_error = Signal()

        self.submodules.outfifo = SyncFIFO(width=len(self.rep.data), depth=8)

        self.sync += [
            If(self.outfifo.writable & self.outfifo.we,
                If(rd_flit == size_rd_port.dat_r - 1,
                    rd_flit.eq(0),
                    tag_in_use[rd_ptr].eq(0),
                    data_valid[rd_ptr].eq(0),
                    If(rd_ptr == num_tags - 1,
                        rd_ptr.eq(0)
                    ).Else(
                        rd_ptr.eq(rd_ptr + 1)
                    )
                ).Else(
                    rd_flit.eq(rd_flit + 1)
                ),
            ),
            If(hmc_port.cmd_ready & hmc_port.cmd_valid,
                tag_in_use[wr_ptr].eq(1),
                If(wr_ptr == num_tags - 1,
                    wr_ptr.eq(0)
                ).Else(
                    wr_ptr.eq(wr_ptr + 1)
                )
            ),
            If(hmc_port.rd_data_valid,
                prev_tag.eq(hmc_port.rd_data_tag),
                If(wr_flit == size_tag_port.dat_r - 1,
                    data_valid[hmc_port.rd_data_tag].eq(1),
                    wr_flit.eq(0)
                ).Else(
                    wr_flit.eq(wr_flit+1)
                )
            ),
            If(hmc_port.rd_data_valid & (wr_flit > 0) & (hmc_port.rd_data_tag != prev_tag),
                self.burst_order_error.eq(1)
            )
        ]
        self.comb += [
            tag_available.eq(~tag_in_use[wr_ptr])
        ]

        self.comb += [
            hmc_port.cmd.eq(hmc_port.HMC_CMD_RD),
            hmc_port.addr.eq(self.req.addr),
            hmc_port.tag.eq(wr_ptr),
            hmc_port.size.eq(self.req.size),
            hmc_port.cmd_valid.eq(self.req.valid & tag_available),
            self.req.ack.eq(hmc_port.cmd_ready & tag_available),

            size_wr_port.dat_w.eq(self.req.size),
            size_wr_port.adr.eq(wr_ptr),
            size_wr_port.we.eq(self.req.valid & tag_available),

            hmc_port.wr_data.eq(0),
            hmc_port.wr_data_valid.eq(0),

            wr_port.dat_w.eq(Cat(hmc_port.rd_data, hmc_port.rd_data_valid)),
            wr_port.we.eq(hmc_port.rd_data_valid),
            wr_port.adr.eq(Cat(wr_flit, hmc_port.rd_data_tag)),

            rd_port.adr.eq(Cat(rd_flit, rd_ptr)),

            size_rd_port.adr.eq(rd_ptr),
            size_tag_port.adr.eq(hmc_port.rd_data_tag)
        ]

        self.comb += [
            self.outfifo.din.eq(rd_port.dat_r),
            self.outfifo.we.eq(data_valid[rd_ptr]),
            self.rep.data.eq(self.outfifo.dout),
            self.rep.valid.eq(self.outfifo.readable),
            self.outfifo.re.eq(self.rep.ack)
        ]

        # stats
        self.stall_cycles = Signal(32)
        self.sync += If(self.req.valid & ~tag_available,
            self.stall_cycles.eq(self.stall_cycles + 1)
        )

#=======================================


import unittest
import random
from migen.fhdl import verilog

class HMCReorderBufferTestBench(Module):
    def __init__(self, num_requests):
        self.port = HMCPort(addr_width=34, size_width=4, data_width=128)
        self.port.hmc_data = [i for i in range(num_requests+8)]
        self.submodules.dut = HMCReorderBuffer(self.port)
        self.num_requests = num_requests

    def gen_cmds(self):
        for i in range(self.num_requests):
            yield self.dut.req.addr.eq(i << 4)
            yield self.dut.req.size.eq(i % 8 + 1)
            yield self.dut.req.valid.eq(1)
            yield
            while not (yield self.dut.req.ack):
                yield
            yield self.dut.req.valid.eq(0)

    def gen_data(self, tc):
        for i in range(self.num_requests):
            for j in range(i % 8 + 1):
                yield self.dut.rep.ack.eq(1)
                yield
                while not ((yield self.dut.rep.ack) and (yield self.dut.rep.valid)):
                    # yield self.dut.rep.ack.eq(random.randrange(2))
                    yield
                data = (yield self.dut.rep.data)
                tc.assertEqual(data, i + j)


class HMCReorderBufferTestCase(unittest.TestCase):
    def setUp(self):
        self.tb = HMCReorderBufferTestBench(num_requests = 128)

    def test_to_verilog(self):
        verilog.convert(self.tb)

    def test_mem(self):
        @passive
        def gen_timeout(cycles):
            time = 0
            while time < cycles:
                yield
                time += 1
            self.fail("Timeout")

        run_simulation(self.tb, [self.tb.gen_cmds(), self.tb.gen_data(self), *self.tb.port.get_simulators(), gen_timeout(10000)], vcd_name="HMCReorderBufferUnitTest.vcd")
