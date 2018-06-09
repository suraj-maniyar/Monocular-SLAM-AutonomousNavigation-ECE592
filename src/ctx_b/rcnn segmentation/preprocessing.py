#!/usr/bin/env python3

"""
Code for preprocessing and loading image and label data.
"""

import os
import sys
from os.path import isfile
from typing import List, Generator, Tuple, Sequence

import numpy as np
from PIL import Image


def read_object_classes(classes_map_filename: str) -> Tuple[List[Tuple[float, float, float]], List[str]]:
    """
    Reads an index of object classes and their corresponding names and colors.
    Each line of the file has 5 elements: R,G,B values as floats, an integer ID, and a name as a string.
    :param classes_map_filename: The filename storing the index
    :return: a tuple of 4 items:
        1. an array of ID -> category color as RGB tuple (in [0, 255])
        2. a dictionary of category color (as an RGB tuple) -> ID
        3. an array of ID -> category name
        2. a dictionary of category name -> ID
    """
    format_description = "Each line should contain 5 elements: (float R, float G, float B, int ID, str Name)."
    ids = set()
    ids_to_cols = {}
    ids_to_names = {}
    with open(classes_map_filename, 'r') as classes_file:
        for line in classes_file:
            try:
                vals = line.split()
                if len(vals) == 0:
                    continue
                elif len(vals) == 2:
                    has_cols = False
                    category_num = int(vals[0])
                    category_name = vals[1]
                elif len(vals) == 5:
                    has_cols = True
                    rgb = tuple([int(255 * float(s)) for s in vals[:3]])
                    category_num = int(vals[3])
                    category_name = vals[4]
                else:
                    raise ValueError("Category map must have either 2 or 5 columns")

                if category_num < 0:
                    continue

                # check for duplicate categories
                if category_num in ids:
                    sys.stderr.write("A category with this number (%d) already exists.\n" % category_num)
                    continue

                ids.add(category_num)
                ids_to_names[category_num] = category_name
                if has_cols:
                    ids_to_cols[category_num] = rgb

            except (ValueError, IndexError) as e:
                sys.stderr.write("%s %s\n" % (format_description, e))
                continue

    max_id = max(ids)
    category_colors = [tuple()] * (max_id + 1)
    category_names = [""] * (max_id + 1)
    for cat_id in ids:
        category_names[cat_id] = ids_to_names[cat_id]
        if has_cols:
            category_colors[cat_id] = ids_to_cols[cat_id]

    return category_colors, category_names


def image_to_np_array(img_filename: str, float_cols: bool = True) -> np.ndarray:
    """
    Reads an image into a numpy array, with shape [height x width x 3]
    Each pixel is represented by 3 RGB values, either as floats in [0, 1] or as ints in [0, 255]
    :param img_filename: The filename of the image to load
    :param float_cols: Whether to load colors as floats in [0, 1] or as ints in [0, 255]
    :return: A numpy array containing the image data
    """
    img = Image.open(img_filename)
    img.load()
    if float_cols:
        data = np.asarray(img, dtype="float32") / 255.0
    else:
        data = np.asarray(img, dtype="uint8")
    return data


def np_array_to_image(array: np.ndarray, output_filename: str):
    """
    Saves a numpy array to an image.
    :param array: A numpy array of shape (height x width x 3), representing an RGB image. Its values should be in the
    range [0,1].
    :param output_filename: The filename in which to store the image.
    """
    img = Image.fromarray((array * 255).astype(dtype=np.uint8), mode='RGB')
    img.save(output_filename)


def labels_to_np_array(lab_filename: str) -> np.ndarray:
    """
    Reads an image of category labels as a numpy array of category IDs.
    NOTE: The image data must already be in a color palette such that color # corresponds to label ID.
    The "Playing for Data" dataset is configured in this way (http://download.visinf.tu-darmstadt.de/data/from_games/)
    :param lab_filename: The filename of the label image to load
    :return: A numpy array containing the label ID for each pixel
    """
    img = Image.open(lab_filename)
    img.load()
    data = np.asarray(img, dtype="uint8")
    return data


def text_labels_to_np_array(lab_filename: str) -> np.ndarray:
    """
    Reads a text file representing a set of labels for an image, and converts it into a numpy array.
    The text file must be organized into rows, and space-separated columns. Each element is a number corresponding to
    a label number for that pixel.
    NOTE: Numbers must be at least 0.
    :param lab_filename: The name of the label file
    :return: A numpy array of shape (height x width) containing numerical labels.
    """
    label_file = open(lab_filename, 'r')
    labels = [list(map(lambda n: max(0, int(n)), l.split())) for l in label_file.readlines()]
    return np.array(labels, dtype=np.int8)


def save_labels_array(labels: np.ndarray, output_filename: str, colors: List[Tuple[float, float, float]]):
    """
    Saves a numpy array of labels to an paletted image.
    :param colors: An array of colors for each index. Should correspond to label ID's in 'labels'
    :param labels: A 2D array of labels
    :param output_filename: The filename of the image to output
    """
    img = Image.fromarray(obj=labels, mode="P")
    # palette is a flattened array of r,g,b values, representing the colors in the palette in order.
    palette = []
    for c in colors:
        palette.extend(c)
    img.putpalette(palette)
    img.save(output_filename)


def get_patch(array: np.ndarray, center: Sequence[int], patch_size: int) -> np.ndarray:
    """
    Returns a square 2D patch of an array with a given size and center. Also returns other dimensions of the array,
    uncropped.
    NOTE: does not do bounds checking.
    :param array: A numpy array
    :param center: The coordinates of the center, as a list or array of length 2
    :param patch_size: A single number representing the width and height of the patch.
    :return: A square patch of the image with the given center and size.
    """
    rounded_width = int(patch_size / 2)
    return array[center[0] - rounded_width: center[0] + rounded_width + 1,
           center[1] - rounded_width: center[1] + rounded_width + 1]


def interleave_images(images, stride):
    """
    Interleaves a series of images with the given stride. The pixels of each image correspond to one out of `stride`
    pixels of every row and column of the output image.
    The top left corner of each image in `images` corresponds to a pixel in [0:stride, 0:stride] in the output,
    in row-major order.
    That is, if the stride is 2, the images' corners will be at pixels [(0,0), (0,1), (1,0), (1,1)] in the output,
    respectively.
    If the image higher dimensions (e.g. 3 RGB channels), these must have the same sizes across all images. Only the
    first two dimensions are interleaved when creating output.

    e.g. `images` =
                    [ [[ 1,  2,  3,  4],
                       [ 5,  6,  7,  8],
                       [ 9, 10, 11, 12]],

                      [[13, 14, 15, 16],
                       [17, 18, 19, 20],
                       [21, 22, 23, 24]],

                      [[25, 26, 27, 28],
                       [29, 30, 31, 32],
                       [33, 34, 35, 36]],

                      [[37, 38, 39, 40],
                       [41, 42, 43, 44],
                       [45, 46, 47, 48]]

         `stride` = 2

         output is:

         [[ 1, 13,  2, 14,  3, 15,  4, 16],
          [25, 37, 26, 38, 27, 39, 28, 40],
          [ 5, 17,  6, 18,  7, 19,  8, 20],
          [29, 41, 30, 42, 31, 43, 32, 44],
          [ 9, 21, 10, 22, 11, 23, 12, 24],
          [33, 45, 34, 46, 35, 47, 36, 48]]

    :param images: A sequence of images of length (stride * stride).
    :param stride: The space between pixels of the same input image in the output.
    :return: An array with width and height equal to the sum of the width and height of the input images, respectively.
    The array will contain the interleaved pixels of the input images.
    """
    h = sum([i.shape[0] for i in images[::stride]])
    w = sum([i.shape[1] for i in images[:stride]])
    # preserve higher-order shape of input
    shape = (h, w) + images[0].shape[2:]
    output = np.zeros(shape=shape)
    i = 0
    for dy in range(stride):
        for dx in range(stride):
            output[dy::stride, dx::stride] = images[i]
            i += 1
    return output


def get_sorted_files_in_folder(folder: str, extension: str = None) -> List[str]:
    """
    Returns a sorted list of all the non-hidden files in a folder, possibly filtered by extension.
     (Note: the search isn't recursive, only the files directly in the given folder are returned)
    :param folder: The folder in which to search
    :param extension: Optional, specifies an extension that all filenames must end with.
    :return: A list of filenames "folder_path/file_name"
    """
    files = [os.path.join(folder, f) for f in os.listdir(folder) if
             isfile(os.path.join(folder, f)) and not f.startswith('.')]
    if extension is not None:
        files = filter(lambda f: f.endswith(extension), files)
    return sorted(files)


# define a type alias for the output of these iterators
DataSetIter = Generator[Tuple[np.ndarray, np.ndarray, str], None, None]


def from_games_dataset(data_dir: str, data_fraction: float = None, num_samples: int = None) -> DataSetIter:
    """
    Iterates over images and labels in the "Playing for Data" dataset from
    <download.visinf.tu-darmstadt.de/data/from_games/>.

    The given directory must have two folders, "images" and "labels", that contain images and corresponding labels,
    respectively.
    Images and labels should both be .png images, with labels being paletted PNGs.
    Each image and corresponding label must have the same filename.

    :param data_dir: The directory in which the data is stored.
    :param data_fraction: Optional - what fraction of the data to return. Can be used to partition data into training
    and
    testing. Can be negative to only return data at the end of the list. e.g. a value of 0.3 will return the first
    30% from the list of files, while -0.2 will return the last 20%.
    :param num_samples: Optional - how many data samples to return. Overrides data_fraction.
    :return: Yields a series of tuples (image, labels, img_id).
        - Image is a numpy array of shape (height x width x 3).
        - Labels is a numpy array of shape (height x width).
        - img_id is a string, identifying the image-label pair.
    """
    labels_dir = os.path.join(data_dir, 'labels')
    images_dir = os.path.join(data_dir, 'images')

    labels = get_sorted_files_in_folder(labels_dir)
    images = get_sorted_files_in_folder(images_dir)
    train_files = list(zip(labels, images))

    # if specified, only choose subset of training data
    if data_fraction is not None and num_samples is None:
        num_samples = int(len(train_files) * data_fraction)
    if num_samples is not None:
        train_files = train_files[:num_samples]

    for label_f, image_f in train_files:
        img_id = os.path.basename(image_f).split('.')[0]
        label_id = os.path.basename(label_f).split('.')[0]
        print("Current image:", os.path.basename(image_f))
        if img_id != label_id:
            print("UNEQUAL IMAGE AND LABEL NAMES!")
        image = image_to_np_array(image_f)
        labels = labels_to_np_array(label_f)
        yield image, labels, img_id


def stanford_bgrounds_dataset(data_dir: str, data_fraction: float = None, num_samples: int = None) -> DataSetIter:
    """
    Iterates over images and labels in the Stanford Background dataset from
    <http://dags.stanford.edu/projects/scenedataset.html>.

    The given directory must have two folders, "images" and "labels", that contain images and corresponding labels,
    respectively.
    Images should be .jpg images, and labels should be text files with the same ID as the image, but with the
    extension ".regions.txt." Text files should contain category labels as a series of space-separate rows of digits.

    :param data_dir: The directory in which the data is stored.
    :param data_fraction: Optional - what fraction of the data to return. Can be used to partition data into training
    and
    testing. Can be negative to only return data at the end of the list. e.g. a value of 0.3 will return the first
    30% from the list of files, while -0.2 will return the last 20%.
    :param num_samples: Optional - how many data samples to return. Overrides data_fraction.
    :return: Yields a series of tuples (image, labels, img_id).
        - Image is a numpy array of shape (height x width x 3).
        - Labels is a numpy array of shape (height x width).
        - img_id is a string, identifying the image-label pair.
    """
    labels_dir = os.path.join(data_dir, 'labels')
    images_dir = os.path.join(data_dir, 'images')

    labels = get_sorted_files_in_folder(labels_dir, extension='.regions.txt')
    images = get_sorted_files_in_folder(images_dir)
    train_files = list(zip(labels, images))

    # if specified, only choose subset of training data
    if data_fraction is not None and num_samples is None:
        num_samples = int(len(train_files) * data_fraction)
    if num_samples is not None:
        if num_samples >= 0:
            train_files = train_files[:num_samples]
        else:
            train_files = train_files[num_samples:]

    for label_f, image_f in train_files:
        if os.path.basename(label_f).split('.')[0] != os.path.basename(image_f).split('.')[0]:
            print("UNEQUAL IMAGE NAMES!", label_f, image_f)
        img_id = os.path.basename(label_f).split('.')[0]
        image = image_to_np_array(image_f)
        labels = text_labels_to_np_array(label_f)
        yield image, labels, img_id


# list of datasets for which we have iterators
FROM_GAMES = 'from-games'
SIFT_FLOW = 'sift-flow'
STANFORD_BGROUND = 'stanford-bground'
DATASETS = {FROM_GAMES: from_games_dataset, SIFT_FLOW: None, STANFORD_BGROUND: stanford_bgrounds_dataset}
