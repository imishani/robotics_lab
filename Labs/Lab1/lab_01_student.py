import numpy as np
import pickle
import os
import time
import glob

logdir_prefix = 'lab-01'

data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../data')


import pickle

data = []

for file in glob.glob(data_path + '/*/*.pkl'):
    with open(file, 'rb') as h:
        data.append(pickle.load(h))

i = 5
