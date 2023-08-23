from migen import *
from migen.fhdl import verilog

from .pico import PicoPlatform

import unittest
import logging

class PicoHMCTestBench(Module):
    def __init__(self):
        self.num_ports = 1
        self.submodules.dut = PicoPlatform(self.num_ports)
        self.cmd_q = [list() for _ in range(self.num_ports)]
        self.dat_w_q = [list() for _ in range(self.num_ports)]
        self.dat_r_d = [dict() for _ in range(self.num_ports)]
        self.rd_sz_d = [dict() for _ in range(self.num_ports)]

    @passive
    def gen_cmds(self):
        while True:
            for p in range(self.num_ports):
                if self.cmd_q[p]:
                    cmd, size, addr, tag = self.cmd_q[p][0]
                    yield self.dut.HMCports[p].cmd.eq(cmd)
                    yield self.dut.HMCports[p].size.eq(size)
                    yield self.dut.HMCports[p].addr.eq(addr)
                    yield self.dut.HMCports[p].tag.eq(tag)
                    yield self.dut.HMCports[p].cmd_valid.eq(1)
                else:
                    yield self.dut.HMCports[p].cmd_valid.eq(0)
            yield
            for p in range(self.num_ports):
                if (yield self.dut.HMCports[p].cmd_ready) and (yield self.dut.HMCports[p].cmd_valid):
                    self.cmd_q[p].pop(0)

    @passive
    def gen_dat_w(self):
        while True:
            for p in range(self.num_ports):
                if self.dat_w_q[p]:
                    yield self.dut.HMCports[p].wr_data.eq(self.dat_w_q[p][0])
                    yield self.dut.HMCports[p].wr_data_valid.eq(1)
                else:
                    yield self.dut.HMCports[p].wr_data_valid.eq(0)
            yield
            for p in range(self.num_ports):
                if (yield self.dut.HMCports[p].wr_data_ready) and (yield self.dut.HMCports[p].wr_data_valid):
                    self.dat_w_q[p].pop(0)

    @passive
    def gen_dat_r(self):
        while True:
            for p in range(self.num_ports):
                if (yield self.dut.HMCports[p].rd_data_valid):
                    tag = (yield self.dut.HMCports[p].rd_data_tag)
                    if tag not in self.dat_r_d[p]:
                        self.dat_r_d[p][tag] = []
                    self.dat_r_d[p][tag].append((yield self.dut.HMCports[p].rd_data))
            yield

    def read(self, port, addr, size, tag):
        self.cmd_q[port].append((self.dut.HMCports[port].HMC_CMD_RD, size, addr, tag))
        while tag not in self.dat_r_d[port] or len(self.dat_r_d[port][tag]) < size:
            yield
        return self.dat_r_d[port].pop(tag)

    def write(self, port, addr, size, data, tag=None):
        self.dat_w_q[port].extend(data)
        if tag:
            self.cmd_q[port].append((self.dut.HMCports[port].HMC_CMD_WR_NP, size, addr, tag))
            while tag not in self.dat_r_d[port]:
                yield
            del self.dat_r_d[port][tag]
        else:
            self.cmd_q[port].append((self.dut.HMCports[port].HMC_CMD_WR, size, addr, 0))

    def selftest(self, result):
        # HMC_CMD_WR
        addr = 0
        for i in range(1,9):
            yield from self.write(port=0, addr=(addr << 4), size=i, data=[(addr + j)<<4 for j in range(i)])
            addr += i
        # HMC_CMD_WR_NP
        addr = 0
        for i in range(1,9):
            yield from self.write(port=0, addr=addr<< 4, size=i, data=[0x42000000 + (addr + j)<<4 for j in range(i)], tag=i)
            addr += i
        # HMC_CMD_RD
        addr = 0
        for i in range(1,9):
            rd_data = yield from self.read(port=0, addr=addr<< 4, size=i, tag=i)
            result.extend(rd_data)
            addr += i


class PicoHMCTestCase(unittest.TestCase):
    def setUp(self):
        self.tb = PicoHMCTestBench()

    def test_to_verilog(self):
        self.tb.dut.export(Module())

    def test_mem(self):
        @passive
        def gen_timeout(cycles):
            time = 0
            while time < cycles:
                yield
                time += 1
            self.fail("Timeout")
        logging.basicConfig(level=logging.INFO)

        result = []
        generators = self.tb.dut.getSimGenerators()
        generators['sys'].extend([self.tb.selftest(result), self.tb.gen_cmds(), self.tb.gen_dat_r(), self.tb.gen_dat_w(), gen_timeout(1000)])
        run_simulation(self.tb, generators, vcd_name="pico_tb.vcd")
        for i, x in enumerate(result):
            self.assertEqual(x, 0x42000000 + i << 4)

if __name__ == '__main__':
    unittest.main()
