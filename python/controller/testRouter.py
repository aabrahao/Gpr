import numpy as np
import Numeric as nm
import Router as rt
from Benchmark import benchmark

def main():
    n = 20
    x = 100*nm.random(n)
    y = 100*nm.random(n)
    
    nodes1 = benchmark( rt.solve, x,y,open_ended=False )
    rt.display(x, y, nodes1)

    nodes2 = benchmark( rt.solve, x,y,open_ended=True )
    rt.display(x, y, nodes2)

    rt.plotBegin(x,y)
    rt.plotRoute(x,y,nodes1,color='red')
    rt.plotRoute(x,y,nodes2,color='blue')
    rt.plotEnd()
    
if __name__ == "__main__":
    main()