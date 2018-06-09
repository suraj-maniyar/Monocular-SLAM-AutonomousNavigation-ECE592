# Segmentation as a Scene Labeling task with Recurrent Convolutional Neural Networks
Experiments with Scene Labeling using Recurrent Convolutional Neural Networks on two different segmentation tasks:
1. Formainifera image segmentation to different chambers and apertures
2. Segmentation of Unity simulated enviroments to safe and unsafe zones

## Overview
This project implements a "Recurrent Convolutional Neural Network" (rCNN) for scene labeling,
as seen in [Pinheiro et al, 2014](http://www.jmlr.org/proceedings/papers/v32/pinheiro14.pdf).

### Summary of files
    README.md           -- README file
    category_maps/      -- Text files containing category info for "Stanford"  & "Data from Games" datasets
    model.py            -- Code for rCNN model
    preprocessing.py    -- Code for processing input data
    requirements.txt    -- Lists python package requirements
    train.py            -- Script for training and evaluating model

## Installation
 1. Clone or download this repository to your computer:

    ```git clone https://github.com/jerisalan/RCNN-Segmentation.git```

 2. Install the necessary Python requirements through `pip`:

    ```pip install -r requirements.txt```

    This project only requires Tensorflow and PIL for image manipulation. TensorFlow v1.3.0 has been used for training and testing but it should work with higher versions too without any issue.

 3. Download a dataset with which to use the model.
 We used the [Stanford Background Dataset](http://dags.stanford.edu/projects/scenedataset.html) which has 700+ 320x240 images and 8 classes.
 The code also works with the [Data from Games](https://download.visinf.tu-darmstadt.de/data/from_games/)
 dataset, which has 25,000 1914 × 1052 with 38 categories.

    To use another dataset, make sure it is organized similarly to one of the above two, and specify while training and testing which dataset it is "mimicking".
  Specifically, both datasets had the data in one folder, with subfolders "labels" and "images" for labels and images, respectively.
  The stanford dataset had labels in the format of space-separated digits in a text file, while the "Data from  Games" dataset had labels in the form of paletted images,
   where each color corresponds to a different label.

  4. Generate a text file that maps colors to category numbers nad labels. Each line of the file has five space-separated values:


      R  G  B category_num category_id

   R,G,B values should be in the range [0,1].
   Category files for the Stanford Background and Data From Games datasets are provided in the folder `category_maps`.

## Running

### Training
For training, use the `train.py` script with the `--training` flag. The following command trains the model on the Stanford dataset:

    python3 train.py --training --dataset stanford-bground --category_map category_maps/stanford_bground_categories.txt --data_dir train_data/ --model_save_path train_model_rcv/


Running `train.py -h` will show additional parameters for the script, including different hyperparameters.

### Testing
For testing, use the `train.py` script without the `--training` flag.
This script will get per-class accuracies for each image, as well as output predicted labels as image files.
The following command loads a saved model and evaluates accuracy on the stanford data set:

    python3 train.py --model_load_path train_model_rcv/ --category_map category_maps/stanford_bground_categories.txt --dataset stanford-bground --data_dir test_data/ --output_dir test_output_rcv/


This outputs per-clas accuracies for each layer of the recurrent rCNN, and also saves predicted labels for each layer.

## Credits

The code for the original project may be found at: https://github.com/NP-coder/CLPS1520Project
