import os

def run(script):
    print('----------------------------------------------')
    print(f'Runing: {script}.py ...')
    os.system(f'python3 {script}.py')
    print(f'{script}.py done!')
    print('----------------------------------------------')

def main():
    run('generateDems')
    run('generateHydrology')
    run('generateRadiationField')
    run('generateMCDM')
    run('plotAll')
    print('All done!')    

if __name__ == "__main__":
    main()
