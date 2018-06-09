# Overview 

This module deals with the Semantic Segmentaton of a construction site scene into 3 classes - Ground, Obstacles, Sky.  

## Getting Started


### Prerequisites

```
OpenCV 2.4.13 with ffmpeg
ROS Kinetic
```

### Installation

Clone the repository by running:

```
$ git clone https://github.ncsu.edu/aros/ece592sp18-AutonomousNavigation.git
```

Compile the Caffe framework 

```
$ cd src/ctx_a/ENet/caffe-enet
$ mkdir build && cd build
$ cmake ..
$ make all -j8 && make pycaffe
```

Add the caffe path to PYTHONPATH

```
$ export PYTHONPATH="$CAFFE_PATH/python:$PYTHONPATH"
```

### Preparation

Copy the training images into `src/ctx_a/ENet/dataset/images`
Copy the annotations into `src/ctx_a/ENet/dataset/labels`

Run the create_lists scripts to generate a list of training images with their corresponding labels.
(Note: Training filenames and annotation filenames must be in the same order as they will be sorted and matched.)

```
$ cd src/ctx_a
$ python ENet/dataset/create_lists.py
```

This generates a file called `file_list.txt`


### Training ENet

First, create the prototxt file `enet_train_encoder.prototxt` by running:

```
$ cd src/ctx_a
$ python ENet/scripts/create_enet_prototxt.py --source ENet/dataset/file_list.txt --mode train_encoder

```
In the `data` layer, in `enet_train_encoder.prototxt`, change the `new_height:` and `new_width:` values to the size of the input data.
In the `deconv_encoder6_0_0` layer, in `enet_train_encoder.prototxt`, set `num_output:` value to 3

Train the encoder by running:

```
$ cd src/ctx_a
$ ENet/caffe-enet/build/tools/caffe train -solver ENet/prototxts/enet_solver_encoder.prototxt
```

Create the decoder prototxt file `enet_train_encoder_decoder.prototxt` by running:

```
$ cd src/ctx_a
$ python ENet/scripts/create_enet_prototxt.py --source ENet/dataset/file_list.txt --mode train_encoder_decoder

```

In the `data` layer, in `enet_train_encoder_decoder.prototxt`, change the `new_height:` and `new_width:` values to the size of the input data.
In the `deconv_encoder6_0_0` layer, in `enet_train_encoder.prototxt`, set `num_output:` value to 3

Train the decoder by running:

```
$ cd src/ctx_a
$ ENet/caffe-enet/build/tools/caffe train -solver ENet/prototxts/enet_solver_encoder_decoder.prototxt -weights ENet/weights/snapshots_encoder/NAME.caffemodel
```

Replace `NAME` with the best encoder model.

### Inference ENet

The Batch Normalisation layers in ENet shift the input feature maps according to their mean and variance
statistics for each mini batch during training. At test time we must use the statistics for the entire dataset.
For this reason run __compute_bn_statistics.py__ to calculate the new weights called __test_weights.caffemodel__:

	$ cd src/ctx_a
	$ python ENet/scripts/compute_bn_statistics.py 	ENet/prototxt/enet_train_encoder_decoder.prototxt \
						        ENet/weights/snapshots_decoder/NAME.caffemodel \
						        ENet/weights/weights_bn/ 

The script saves the __bn-weights__ in the output directory `ENet/weights/weights_bn/bn_weights.caffemodel`.

For inference batch normalization and dropout layer can be merged into convolutional kernels, to
speed up the network. You can do this by running:

	$ python ENet/scripts/BN-absorber-enet.py 	--model ENet/prototxts/enet_deploy.prototxt \
					                --weights ENet/weights/bn_weights.caffemodel \
					                --out_dir ENet/final_model_weigths/

It also deletes the corresponding batch normalization and dropout layers from the prototxt file. The final model (prototxt file) and weights are saved in the folder __final_model_and_weights__. 

The input images for inferencing must be in the folder `src/ctx_a/output`

Setup the ROS worspace to read from RGB and depth topics and save the images into `src/ctx_a/output` by running:

```
$ cd src/ctx_a/cat
$ catkin_make
```

Run the ROS package using: 

```
$ rosrun foopkg test.py
```

The ROS package subscribes to RGB and depth streams and publishes the following topics:

```
/camera/ctx_a/segmentation_output
/camera/ctx_a/segmentation_output_depth
/camera/ctx_a/line_output
``` 

To segment from a live camera feed, modify the location from where to read images live in `test_cpp_rt.sh` and run:

```
$ ./ENet/scripts/test_cpp_rt.sh
```

To segment from a video, modify the location from where to read video in `test_cpp_rt.sh` and run:

```
$ ./ENet/scripts/test_cpp_vid.sh
```

The test segmentation C++ soure for real-time can be modified at `ENet/caffe-enet/examples/ENet_with_C++/test_segmentation_rt.cpp`
The test segmentation C++ soure for reading from video can be modified at `ENet/caffe-enet/examples/ENet_with_C++/test_segmentation_vid.cpp`

Copy the file to be built as `ENet/caffe-enet/examples/ENet_with_C++/test_segmentation.cpp`
Build the code by running:

```
$ cd src/ctx_a/ENet/caffe-enet/build/examples
$ make test_segmentation
```

Move the compiled binaries to the corresponding folders inside `ENet/caffe-enet/build/examples/ENet_with_C++`

You should be able to produce results as seen below:

![Alt text](ENet/example_image/demo.png?raw=true "demo") 

