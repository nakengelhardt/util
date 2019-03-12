

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

def get_simulators(module, name, *args, **kwargs):
    simulators = []
    if hasattr(module, name):
        simulators.append(getattr(module, name)(*args, **kwargs))
    for _, submodule in module._submodules:
            for simulator in get_simulators(submodule, name, *args, **kwargs):
                    simulators.append(simulator)
    return simulators