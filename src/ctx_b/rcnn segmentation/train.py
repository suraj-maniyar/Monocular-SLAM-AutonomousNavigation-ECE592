#!/usr/bin/env python3

"""
a
"""

import argparse
import os
import random
import time

import numpy as np
import tensorflow as tf

from model import RCNNModel, save_model, restore_model
from preprocessing import read_object_classes, DATASETS, FROM_GAMES, save_labels_array, np_array_to_image, \
    interleave_images


class AccuracyCounter:
    """
    This class stores per-class accuracy counts for some task, such as testing a classifier.
    """

    def __init__(self, num_classes):
        self.num_classes = num_classes
        self.class_correct_counts = np.zeros(self.num_classes)
        self.class_total_counts = np.zeros(self.num_classes)

    def add_sample(self, predicted_labels, true_labels):
        """
        Adds a sample to the accuracy counts.
        :param predicted_labels: An array of predicted labels
        :param true_labels: An array of the true labels, in the same shape as `predicted_labels`
        :return: The % accuracy for the given sample across all classes
        """
        correct_class_labels = np.equal(predicted_labels, true_labels)
        for c in range(self.num_classes):
            current_class_labels = np.equal(true_labels, c)
            self.class_total_counts[c] += np.sum(current_class_labels)
            self.class_correct_counts[c] += np.sum(current_class_labels * correct_class_labels)
        return np.mean(correct_class_labels)

    def get_results(self):
        """
        Get results on the accuracy for the samples stored in this counter object.
        :return: A tuple of overall accuracy, per-class accuracy, and total counts for each class
        """
        class_accuracies = self.class_correct_counts / self.class_total_counts
        total_accuracy = np.sum(self.class_correct_counts) / np.sum(self.class_total_counts)
        return total_accuracy, class_accuracies, self.class_total_counts


def run_model_on_image(sess, model, image, labels, is_training: bool = False):
    """
    Given an image and labels, runs the model and returns output. A training step is also optionally run.
    :param sess: The session within which to run the model
    :param model: The model to run
    :param image: The input RGB image, a numpy array of shape height x width x 3.
    :param labels: An array of labels corresponding to the image, of shape height x width
    :param is_training: If true, trains the model to minimize the loss.
    :return: The results of running the model. Specifically, a tuple containing logits for each layer, the numerical
    loss, and an optional element corresponding to the training step.
    """
    # check for invalid labels in image
    error_classes = np.sum(np.less(labels, 0) + np.greater_equal(labels, model.num_classes))
    if error_classes > 0:
        print("ERROR - Incorrect labels in image:", labels)
        return

    ops_to_run = model.logits + [model.loss]
    if is_training:
        ops_to_run.append(model.train_step)

    h, w = labels.shape
    input_image = np.append(image, np.zeros(shape=[h, w, model.num_classes], dtype=np.float32), axis=2)
    feed_dict = {model.inpt: [input_image], model.output: [labels]}
    return sess.run(ops_to_run, feed_dict=feed_dict)


def run_model(sess, model, dataset_iter, num_epochs=1, training=False, save_path=None, color_map=None,
              output_dir=None):
    """
    Runs a given model on the given data set. Outputs per-class accuracies for the given data, and optionally trains
    the model as well.
    :param sess: A tensorflow session in which to run the model
    :param model: A given rCNN model
    :param dataset_iter: An iterator over tuples of (image, labels, img_id) in the dataset
    :param num_epochs: Number of epochs to run over the entire dataset
    :param training: Whether to run a training step for each sample, or to only test.
    :param save_path: Path in which to store saved model. (Saves every epoch)
    :param color_map: Optional, an array of colors for each class label, used to output predicted labels.
    :param output_dir: Optional, directory in which to output predicted labels for each class.
    """
    for i in range(num_epochs):
        print('Running epoch %d/%d...' % (i + 1, num_epochs))
        layer_accuracies = [AccuracyCounter(model.num_classes) for _ in range(model.num_layers)]
        n = 0
        for image, labels, img_id in dataset_iter():
            start_time = time.time()
            n += 1

            # create several shifted copies of the input
            shifted_images = []
            shifted_labels = []
            stride = 2 ** model.num_layers
            for dy in range(stride):
                for dx in range(stride):
                    shifted_images.append(image[dy:, dx:, :])
                    shifted_labels.append(labels[dy:, dx:])

            # get model output for each shifted image/label pair
            op_results = [run_model_on_image(sess, model, i, l, is_training=training) for i, l in
                          zip(shifted_images, shifted_labels)]
            grouped_results = list(zip(*op_results))

            # for each shifted image, logits are in trivial batches of size 1.
            layer_logits = [[shifted_logits[0] for shifted_logits in layer] for layer in
                            grouped_results[:model.num_layers]]
            loss = np.mean(grouped_results[model.num_layers])
            if loss != loss:
                print("Loss is NaN! Stopping training without saving.")
                return
            elapsed_time = time.time() - start_time
            print("%s on image #%d (%s): Loss: %f Elapsed time: %.1f" % (
                "Trained" if training else "Tested", n, img_id, loss, elapsed_time))

            # interlace logits to create labels that have the same size as the input labels
            merged_labels = []
            for l in range(model.num_layers):
                current_stride = 2 ** (l + 1)
                layer_labels = [np.argmax(shifted_logits, axis=2) for shifted_logits in layer_logits[l]]
                # If this is not the final layer, not all shifted outputs are needed.
                # Specifically, if the outputs each correspond to a different pixel in the top left (2^num_layers x
                # 2^num_layers pixels), one only needs the first (2^current_layer x 2 ^ current_layer)
                if l < model.num_layers - 1:
                    layer_labels = [layer_labels[dy * stride + dx] for dy in range(current_stride) for dx in range(current_stride)]
                merged_labels.append(interleave_images(layer_labels, stride=current_stride))

            # accuracy
            for l in range(model.num_layers):
                acc = layer_accuracies[l].add_sample(predicted_labels=merged_labels[l], true_labels=labels)
                print("Accuracy for layer %d: %f" % (l + 1, acc))

            # write outputs to disk
            if output_dir is not None and color_map is not None:
                for l in range(model.num_layers):
                    output_filename = os.path.join(output_dir, img_id + '_test_%d.png' % (l + 1))
                    save_labels_array(merged_labels[l].astype(np.uint8), output_filename, colors=color_map)

        for l in range(model.num_layers):
            total_accuracy, class_accuracies, class_total_counts = layer_accuracies[l].get_results()
            print("Layer %d total accuracy:" % (l + 1), total_accuracy)
            print("Layer %d per-class accuracies:" % (l + 1), class_accuracies)
            print("Layer %d per-class total counts:" % (l + 1), class_total_counts)

        if save_path is not None:
            print("Epoch %i finished, saving trained model to %s..." % (i + 1, save_path))
            save_model(sess, save_path)


def optimize_input(sess, model, labels, step_size=2e-2, num_iterations=100):
    """
    Optimizes input for the given model using gradient descent, in order to match the given output labels.
    This is done by minimizing loss between the given labels and the output from running the generated image through
    the model.
    Each time a gradient descent step fails to improve loss, the step size is halved.
    :param sess: A tensorflow session
    :param model: A given rCNN model
    :param labels: The labels to match, as a numpy array of shape (height x width)
    :param step_size: The initial step size for gradient descent.
    :param num_iterations: Number of gradient descent steps to run.
    :return: The resulting image as an RGB numpy array of shape (height x width x 3)
    """
    # use loss from final layer
    loss = model.errors[-1]
    grad = tf.gradients(loss, model.inpt)[0]
    h, w, = labels.shape
    image = np.random.uniform(low=0.4, high=0.6, size=(h, w, 3))
    # image = np.zeros(shape=(h, w, 3)) + 0.5
    image = np.append(image, np.zeros(shape=[h, w, model.num_classes], dtype=np.float32), axis=2)
    l_prev = None
    for i in range(num_iterations):
        g, l = sess.run([grad, loss], feed_dict={model.inpt: [image], model.output: [labels]})
        if l_prev is not None and l_prev <= l:
            step_size /= 2
        g = g[0]  # we're using a batch size of 1 here
        # normalize gradients
        g /= g.std() + 1e-8
        image[:, :, :3] -= g[:, :, :3] * step_size
        print("Iteration %d, step size: %f, score: %f" % (i, step_size, l))
        l_prev = l
    return image[:, :, :3]


def run_gradient_descent(sess, model, true_labels, output_file=None, category_colors=None):
    """
    Given a set of labels, iteratively optimizes inputs to produce those given labels. Similar to a Google "Deep Dream".
    The produced image, as well as the resulting predicted labels, are returned and optionally saved.
    :param sess: A tensorflow session
    :param model: An rCNN model
    :param true_labels: A given set of labels for an image, of shape (height x width)
    :param output_file: Filename to store resulting image (as well resulting labels)
    :param category_colors: Map from category to color, for storing resulting labels
    :return: A tuple of (resulting image, resulting labels)
    """
    image = optimize_input(sess, model, true_labels)
    if output_file is not None and category_colors is not None:
        np_array_to_image(image, output_filename=output_file)

    ops = run_model_on_image(sess, model, image, true_labels)
    last_logits = ops[model.num_layers - 1][0]
    predicted_labels = np.argmax(last_logits, axis=2)
    predicted_labels = np.kron(predicted_labels, np.ones(shape=[2 ** model.num_layers, 2 ** model.num_layers]))
    if output_file is not None and category_colors is not None:
        save_labels_array(predicted_labels.astype(np.uint8), output_file + 'descent_labels.png', colors=category_colors)
    return image, predicted_labels


def main():
    """
    Trains or evaluates an rCNN model.
    """
    # parse command line arguments
    parser = argparse.ArgumentParser(description='An rCNN scene labeling model.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--dataset', type=str, default=FROM_GAMES, choices=DATASETS.keys(),
                        help='Type of dataset to use. This determines the expected format of the data directory')
    parser.add_argument('--category_map', type=str, help='File that maps colors ')
    parser.add_argument('--data_dir', type=str, help='Directory for image and label data')
    parser.add_argument('--data_fraction', type=float, default=None,
                        help='Fraction of data to train on. If positive, trains on first X images, otherwise trains on '
                             'last X images.')
    parser.add_argument('--hidden_size_1', type=int, default=10, help='First Hidden size for CNN model')
    parser.add_argument('--hidden_size_2', type=int, default=10, help='Second Hidden size for CNN model')
    parser.add_argument('--learning_rate', type=float, default=0.001, help='Learning rate for training CNN model')
    parser.add_argument('--num_epochs', type=int, default=1, help='Number of epochs for training CNN model')
    parser.add_argument('--model_save_path', type=str, default=None,
                        help='Optional location to store saved model in.')
    parser.add_argument('--model_load_path', type=str, default=None,
                        help='Optional location to load saved model from.')
    parser.add_argument('--training', action='store_true', default=False, help='Whether or not to train model.')
    parser.add_argument('--output_dir', type=str, default=None, help='Directory in which to save test output images.')
    parser.add_argument('--dry_run', action='store_true', default=False,
                        help='If true, only trains on one image, to test the training code quickly.')
    parser.add_argument('--fix_random_seed', action='store_true', default=False,
                        help='Whether to reset random seed at start, for debugging.')

    args = parser.parse_args()

    if args.fix_random_seed:
        random.seed(0)

    # load class labels
    category_colors, category_names = read_object_classes(args.category_map)
    num_classes = len(category_names)

    # create function that when called, provides iterator to an epoch of the data
    dataset_func = DATASETS[args.dataset]
    if args.dry_run:
        def dataset_epoch_iter():
            return dataset_func(args.data_dir, num_samples=1)
    else:
        def dataset_epoch_iter():
            return dataset_func(args.data_dir, data_fraction=args.data_fraction)

    model = RCNNModel(args.hidden_size_1, args.hidden_size_2, num_classes,
                      args.learning_rate, num_layers=2)

    sess = tf.Session()
    init = tf.initialize_all_variables()
    sess.run(init)
    if args.model_load_path is not None:
        restore_model(sess, args.model_load_path)

    run_model(sess, model, dataset_epoch_iter, num_epochs=args.num_epochs, training=args.training,
              save_path=args.model_save_path, output_dir=args.output_dir, color_map=category_colors)


if __name__ == '__main__':
    main()
