from migen import *

def FullyInitMemory(width, depth, init=None, name=None):
    mem_init = init if init else []
    if (len(mem_init) < depth):
        mem_init.extend([0] * (depth - len(mem_init)))
    return Memory(width, depth, mem_init, name)
