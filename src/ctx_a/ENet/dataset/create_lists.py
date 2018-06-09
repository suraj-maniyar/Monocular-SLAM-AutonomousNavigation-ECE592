import os
import glob
import numpy as np

image_path = "ENet/dataset/images"
label_path = "ENet/dataset/labels"

# Get the training set images and labels

image_names = np.sort(np.array(glob.glob(image_path+'/*.png')))
label_names = np.sort(np.array(glob.glob(label_path+'/*.png')))

for idx_images in range(0,len(image_names)):
    f = open('file_list.txt','a')
    str_1 = image_names[idx_images]
    str_2 = label_names[idx_images]
    print(str_1 +' '+str_2+ '\n')
    f.write(str_1 + ' ' + str_2+'\n')
    f.close()
