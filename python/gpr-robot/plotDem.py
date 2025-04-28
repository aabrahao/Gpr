#!/usr/bin/env python3

import sys
import robot.geometry.geotiff as tif


if len(sys.argv) < 2:
    print('No argument file found!')
    exit()

filename = sys.argv[1]

database = tif.open(filename)
tif.info(database)
tif.show(database)
