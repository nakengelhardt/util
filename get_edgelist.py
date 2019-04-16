from migen import *
from migen.genlib.fifo import *
from migen.genlib.fsm import *

from hmc_reorder_buffer import HMCReorderBuffer
from pico import HMCPort
from recordfifo import InterfaceFIFO

class GetEdgelistHMC(Module):
    def __init__(self, port, vertex_size_in_bits=32):
        # bit widths
        addr_size = len(port.addr)
        max_flit_in_burst = 8
        bytes_per_flit = len(port.rd_data)//8
        assert bytes_per_flit == 16
        vertices_per_flit = len(port.rd_data)//vertex_size_in_bits
        vtx_offset = log2_int(vertex_size_in_bits//8)
        flit_offset = log2_int(len(port.rd_data)//8)
        burst_offset = flit_offset+log2_int(max_flit_in_burst)


        # input signals
        self.req = Record([
            ("start_address", addr_size),
            ("end_address", addr_size),
            ("valid", 1),
            ("ack", 1)
        ])

        # output signals

        self.rep = Record([
            ("vertex_array", len(port.rd_data)),
            ("nvtx", bits_for(vertices_per_flit)),
            ("last", 1),
            ("valid", 1),
            ("ack", 1)
        ])

        # submodules
        self.submodules.ordered_port = HMCReorderBuffer(port)
        self.submodules.length_fifo = SyncFIFO(width=bits_for(max_flit_in_burst*vertices_per_flit), depth=64)
        self.submodules.last_fifo = SyncFIFO(width=1, depth=64)

        self.submodules.outfifo = InterfaceFIFO(layout=[
            ("vertex_array", len(port.rd_data), DIR_M_TO_S),
            ("nvtx", bits_for(vertices_per_flit), DIR_M_TO_S),
            ("last", 1, DIR_M_TO_S),
            ("valid", 1, DIR_M_TO_S),
            ("ack", 1, DIR_S_TO_M)
        ], depth=8)

        # issue requests for edgelist
        addr = Signal(addr_size)
        end_addr = Signal(addr_size)
        length_bytes = Signal(addr_size)
        nvtx_remaining = Signal(addr_size)
        valid = Signal()
        get_new_addr = Signal()
        partial_burst = Signal()

        nflit_in_last_burst = Signal(burst_offset-flit_offset)
        nvtx_in_last_flit = Signal(flit_offset-vtx_offset)
        partial_flit = Signal()
        last_cmd = Signal()

        self.comb += [
            length_bytes.eq(end_addr - addr),
            nvtx_remaining.eq(length_bytes[vtx_offset:]),
            nflit_in_last_burst.eq(length_bytes[flit_offset:burst_offset] + partial_flit),
            partial_burst.eq(nflit_in_last_burst != 0),
            nvtx_in_last_flit.eq(length_bytes[vtx_offset:flit_offset]),
            partial_flit.eq(nvtx_in_last_flit != 0),
            last_cmd.eq(length_bytes <= max_flit_in_burst*bytes_per_flit)
        ]

        self.sync += [
            If(get_new_addr,
                addr.eq(self.req.start_address),
                end_addr.eq(self.req.end_address),
                valid.eq(self.req.valid)
            ).Elif(self.ordered_port.req.valid & self.ordered_port.req.ack,
                addr.eq(addr + max_flit_in_burst*bytes_per_flit),
            ),
        ]

        self.comb += [
            self.req.ack.eq(get_new_addr),
            self.ordered_port.req.addr.eq(addr),
            self.ordered_port.req.valid.eq(valid),
            self.last_fifo.din.eq(last_cmd),
            If(last_cmd & (partial_burst | partial_flit),
                self.length_fifo.din.eq(length_bytes[vtx_offset:burst_offset])
            ).Else(
                self.length_fifo.din.eq(max_flit_in_burst*vertices_per_flit)
            ),
            If(last_cmd & partial_burst,
                self.ordered_port.req.size.eq(nflit_in_last_burst),
            ).Else(
                self.ordered_port.req.size.eq(max_flit_in_burst),
            ),
            self.length_fifo.we.eq(self.ordered_port.req.ack & self.ordered_port.req.valid),
            self.last_fifo.we.eq(self.length_fifo.we),
            If(~valid,
                get_new_addr.eq(1),
            ).Elif(self.ordered_port.req.ack & self.ordered_port.req.valid,
                get_new_addr.eq(last_cmd)
            )
        ]


        # get responses

        nvtx_expected = Signal(bits_for(max_flit_in_burst*vertices_per_flit))
        nvtx_expected_valid = Signal()
        get_new_expected = Signal()
        last_flit = Signal()
        last_burst = Signal()

        self.sync += [
            If(get_new_expected,
                nvtx_expected.eq(self.length_fifo.dout),
                nvtx_expected_valid.eq(self.length_fifo.readable),
                last_burst.eq(self.last_fifo.dout)
            ).Elif(self.ordered_port.rep.valid & self.ordered_port.rep.ack,
                nvtx_expected.eq(nvtx_expected - vertices_per_flit),
            )
        ]

        self.comb += [
            self.length_fifo.re.eq(get_new_expected),
            self.last_fifo.re.eq(get_new_expected),
            last_flit.eq(nvtx_expected <= vertices_per_flit),
            If(self.outfifo.din.ack,
                If(nvtx_expected_valid,
                    get_new_expected.eq(last_flit & self.ordered_port.rep.valid),
                ).Else(
                    get_new_expected.eq(1)
                )
            ),

            self.outfifo.din.vertex_array.eq(self.ordered_port.rep.data),
            self.outfifo.din.valid.eq(self.ordered_port.rep.valid & nvtx_expected_valid),
            self.ordered_port.rep.ack.eq(self.outfifo.din.ack & nvtx_expected_valid),

            If(last_flit,
                self.outfifo.din.last.eq(last_burst),
                self.outfifo.din.nvtx.eq(nvtx_expected)
            ).Else(
                self.outfifo.din.last.eq(0),
                self.outfifo.din.nvtx.eq(vertices_per_flit)
            ),

            self.outfifo.dout.connect(self.rep)

        ]

    @passive
    def gen_selfcheck(self, testcase=None):
        def fail(reason=""):
            if testcase:
                testcase.fail(reason)
            else:
                import logging
                logger = logging.getLogger("sim")
                logger.error(reason)

        yield

        if (yield self.req.valid):
            start = (yield self.req.start_address)
            end = (yield self.req.end_address)
            if end < start:
                fail("Request to get edgelist with invalid range {:x} - {:x}".format(start, end))
            if start % 16 != 0:
                fail("start_address not aligned: {}".format(end))
            if end % 4 != 0:
                fail("end_address not aligned: {}".format(end))


#=======================================


import unittest
import random
from migen.fhdl import verilog

class GetEdgelistHMCTestBench(Module):
    def __init__(self, requests, data):
        self.port = HMCPort(addr_width=34, size_width=4, data_width=128)
        self.port.hmc_data = data
        self.submodules.dut = GetEdgelistHMC(self.port)
        self.requests = requests

    def gen_input(self):
        for start, end in self.requests:
            yield self.dut.req.start_address.eq(start)
            yield self.dut.req.end_address.eq(end)
            yield self.dut.req.valid.eq(1)
            yield
            while not (yield self.dut.req.ack):
                yield
            yield self.dut.req.valid.eq(0)

    def gen_output(self, tc):
        for start, end in self.requests:
            length = (end-start)
            partial_flit = length % 16 != 0
            num_flits = length//16
            if partial_flit:
                num_flits += 1

            for i in range(num_flits):
                yield self.dut.rep.ack.eq(1)
                yield
                while not ((yield self.dut.rep.valid) and (yield self.dut.rep.ack)):
                    # yield self.dut.rep.ack.eq(random.randrange(2))
                    yield
                data = (yield self.dut.rep.vertex_array)
                nvtx = (yield self.dut.rep.nvtx)
                tc.assertEqual(data, start//16 + i)
                if i == num_flits - 1 and partial_flit:
                    tc.assertEqual(nvtx, (length//4) % 4)
                else:
                    tc.assertEqual(nvtx, 4)
                if i == num_flits - 1:
                    tc.assertEqual((yield self.dut.rep.last), 1)
                else:
                    tc.assertEqual((yield self.dut.rep.last), 0)


class GetEdgelistHMCTestCase(unittest.TestCase):
    def setUp(self):
        requests = [(0,8*16), (0, 10*16), (3*16, 3*16+8*16+31*4)]
        self.tb = GetEdgelistHMCTestBench(requests=requests, data=[i for i in range(12*16)])

    def test_to_verilog(self):
        verilog.convert(self.tb)

    def test_sim(self):
        @passive
        def gen_timeout(cycles):
            time = 0
            while time < cycles:
                yield
                time += 1
            self.fail("Timeout")


        run_simulation(self.tb, [self.tb.gen_input(), self.tb.gen_output(self), self.tb.dut.gen_selfcheck(), *self.tb.port.get_simulators(), gen_timeout(10000)], vcd_name="GetEdgelistHMCUnitTest.vcd")
