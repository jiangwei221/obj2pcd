import os
import sys
from subprocess import call
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("models_dir", type=str,
                    help="models dir")
parser.add_argument("output_dir", type=str,
                    help="output dir")
parser.add_argument("sample_density", type=int,
                    help="sample density")
parser.add_argument("--normals", type=int,
                    help="interpolate normals or not")
parser.add_argument("--flip", type=int,
                    help="flip the normals or not") 

args = parser.parse_args()

#sample command:
#python3 convertall.py ./models ./output 200 --normals 1

models_dir = args.models_dir
output_dir = args.output_dir
sample_density = args.sample_density
normal_flag = 1 if (args.normals == None) else args.normals
flip_flag = 0 if (args.flip == None) else args.flip

print('models_dir ' + str(models_dir))
print('output_dir ' + str(output_dir))
print('sample_density ' + str(sample_density))
print('normal_flag ' + str(normal_flag))
print('flip_flag ' + str(flip_flag))


for filename in os.listdir(models_dir):
    if filename.endswith('.obj'):
        print(filename)
        call(["./build/obj2pcd", 
                models_dir+'/'+filename, 
                output_dir+'/'+filename[0:-4]+'.pcd',
                '-sample_density', str(sample_density),
                '-normal_flag', str(normal_flag),
                '-flip_flag', str(flip_flag)])
