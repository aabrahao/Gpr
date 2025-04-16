
import Geotiff as gt
import Visualization as vz

def test(path):
    database = gt.open(path)
    gt.info(database)
    vz.plotDem(database)

test('data/rifle/terrain/masked')
test('data/rifle/terrain/maskedtwi')
