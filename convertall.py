import os
import sys
from subprocess import call

#sample command:
#python3 covertall.py ./models ./output 200

if (len(sys.argv) < 4):
    sys.stderr.write('Usage: models_dir output_dir sample_density optional_normal_flag optional_flip_flag ')
    sys.exit(1)

models_dir = sys.argv[1]
output_dir = sys.argv[2]
sample_density = int(sys.argv[3])
normal_flag = '1'
flip_flag = '0'

if(len(sys.argv) == 5):
    normal_flag = sys.argv[4]

if(len(sys.argv) == 6):
    flip_flag = sys.argv[5]

for filename in os.listdir(models_dir):
    if filename.endswith('.obj'):
        call(["./build/obj2pcd", 
                models_dir+'/'+filename, 
                output_dir+'/'+filename[0:-4]+'.pcd',
                str(sample_density),
                normal_flag,
                flip_flag])
