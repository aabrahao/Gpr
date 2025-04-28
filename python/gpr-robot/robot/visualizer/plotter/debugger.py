import numpy as np
import matplotlib.pyplot as plt

def stats(x, var=None):
    shape = x.shape
    v = x.ravel()
    min = np.nanmin(v)
    max = np.nanmax(v)
    rng = max-min
    avg = np.nanmean(v)
    std = np.nanstd(v)
    med = np.nanmedian(v)
    nans = np.isnan(v).sum()
    infs = np.isinf(v).sum()
    return shape,min,max,rng,avg,std,med,nans,infs

def display(x, var=''):
    shape,min,max,rng,avg,std,med,nans,infs = stats(x)
    if var:
        print(f'{var}:', end=' ')
    print( (f'{shape} > min: {min} max: {max} range: {rng} '
            f'mean: {avg} std: {std} median: {med} '
            f'nans: {nans} infs: {infs}'))    

def axes(title=''):
    fig = plt.figure(figsize=(18,18))
    ax = fig.add_subplot()
    ax.grid(True)
    ax.figure.tight_layout()
    if title:
        ax.set_title(title)
    return ax


def plot(x,var=''):
    display(x, var)
    i = range(len(x))
    ax = axes()
    ax.plot(x)
    imin = np.argmin(x)
    imax = np.argmax(x)
    ax.plot((i[imin]),(x[imin]),'o',color='blue')
    ax.plot((i[imax]),(x[imax]),'o',color='red')
    plt.show()