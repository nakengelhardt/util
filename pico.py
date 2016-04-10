from migen import *
from migen.genlib.record import *

_stream_layout = [
    ("valid", 1, DIR_M_TO_S),
    ("rdy", 1, DIR_S_TO_M),
    ("data", "data_width", DIR_M_TO_S)
]

class PicoStreamInterface(Record):
    def __init__(self, data_width=128):
        Record.__init__(self, set_layout_parameters(_stream_layout, data_width=data_width))

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

def gen_channel_write(channel, words):
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

def gen_channel_read(channel, nwords):
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
    ("PicoClk", 1, DIR_M_TO_S),
    ("PicoRst", 1, DIR_M_TO_S),
    ("PicoAddr", "data_width", DIR_M_TO_S),
    ("PicoDataIn", "data_width", DIR_M_TO_S),
    ("PicoRd", 1, DIR_M_TO_S),
    ("PicoWr", 1, DIR_M_TO_S),
    ("PicoDataOut", "data_width", DIR_S_TO_M)
]

class PicoBusInterface(Record):
    def __init__(self, data_width=128):
        Record.__init__(self, set_layout_parameters(_bus_layout, data_width=data_width))
        for name in [x[0] for x in _bus_layout]:
            getattr(self, name).name_override = name

    def get_ios(self):
        return { getattr(self, name) for name in [x[0] for x in _bus_layout]}
