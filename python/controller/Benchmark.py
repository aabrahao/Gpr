from time import time

def benchmark(func, *args, **kwargs):
    print('Benchmarking... ', end='')
    start = time()
    results = func(*args, **kwargs)
    end = time()
    print(f'{end-start} seconds!')
    return results

