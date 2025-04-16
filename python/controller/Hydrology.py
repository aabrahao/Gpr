
import os
import glob
import Geotiff as gt
import DEM as dm

g_folder = ''
g_project = ''
g_lib = '/usr/local/taudem/'
g_process = 8

def setProject(folder, project):
    global g_folder, g_project
    g_folder = folder
    g_project = project

def project():
    return f'{g_folder}/{g_project}'

def filename(file):
    return file.rsplit('/', 1)[-1]

def run(command):
    print('')
    print('############################################################')
    print(command)
    print('-----------------------------------')
    cmd = f'mpiexec -n {g_process} {g_lib}/{command.lower()} {g_folder}/{g_project}.tif';
    print(cmd)
    os.system(cmd)

def pitExtraction():
    path = project()
    terrain = gt.open(path)
    x,y,z = gt.dem(terrain)
    # Filled pit
    filled = gt.open(path + 'fel')
    xf,yf,zf = gt.dem(filled)
    # Differance
    zp = zf - z
    dm.save(x,y,zp,path + 'pit')

def generate():
    # Remove Pits 
    run('PitRemove')
    # Flow Directions 
    run('D8Flowdir')
    run('DinfFlowdir')
    # Contributing area 
    run('AreaD8')
    run('AreaDinf')
    # Gridnet 
    run('GridNet')
    # PeukerDouglas 
    run('PeukerDouglas')
    # PeukerDouglas stream delineation 
    # Add...
    # Stream Network
    #run('StreamNet')
    # Twi
    run('TWI')
    pitExtraction()

def delete(file):
    if file == project():
        print(f'{file} project kept!')
    else:
        #os.system(f'rm {file}')
        print(f'{file} deleted!')

def find(match):
    return glob.glob(match)

def outputs():
    files = find(f'{g_folder}/{g_project}*.tif')
    files = [file.replace('.tif', '') for file in files]
    files.remove(project())
    return files

def clean():
    files = outputs()
    print('Cleaning...')
    for file in files:
        delete(file)
   
