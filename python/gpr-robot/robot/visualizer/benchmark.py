from time import time

def benchmark(func, *args, **kwargs):
    print('Benchmarking... ', end='')
    start = time()
    results = func(*args, **kwargs)
    end = time()
    print(f'{end-start} seconds!')
    return results

def error(a,v):
    return abs(a-v)/a
