import numpy as np

def main():
    m = 10*np.random.random((4,3))
    m[0,0] = np.nan
    m[-1,-1] = np.nan
    print(m)
    print(m>0.0)
    print(m == np.nan)
    i = np.where(m.flatten() > 0.1)[0]
    j = np.where(np.isnan(m.flatten()))[0]
    k = np.where(np.isnan(m.flatten()) | (m.flatten() > 0.1))
    k = np.where(np.isnan(m) | (m > 0.1))
    print(m.flatten())
    print(i)
    print(j)
    print(k)
    print(len(m))
    print(m.size)

    a = None

    if a:
        print(f'{a} exists!')
    else:
        print(f'{a} is none!')

    if ~a:
        print(f'{a} is none!')
    else:
        print(f'{a} exists!')


if __name__ == "__main__":
    main()