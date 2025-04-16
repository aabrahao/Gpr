import numpy as np
import Numeric as nm

def stratify(data, bins):
    categories = np.digitize(data, bins)
    return categories

def count(categories):
    return categories.max()+1

def layer(data, categories, index):
    stratum = 0.0*data
    i = categories==index
    stratum[i] = data[i]
    return stratum

def layers(data, categories):
    n = count(categories)
    strata = []
    for i in range(n):
        stratum = layer(data, categories, i)
        strata.append(stratum)
    return strata

def extract(Z, tresholds):
    categories = stratify(Z, tresholds)
    return layers(Z, categories)

def qualify(Z, percentiles = (0.25, 0.5, 0.75)):
    strata = extract(Z, np.quantile(Z, percentiles))
    return strata

def mask(Z, levels):
    if nm.scalar(levels):
        return nm.mask(Z, levels)
    maps = []
    for level in levels:
        maps.append( mn.mask(Z, level) )
    return maps