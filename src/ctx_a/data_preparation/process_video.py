# -*- coding: utf-8 -*-
"""
Created on Fri Mar 16 18:46:20 2018

@author: akshay
"""
import cv2
import sys, os
import numpy as np
import argparse

def main(argv):
    
    # Get command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--i', dest='video_path', action='store', type=str, help='Path to the input image')
    parser.add_argument('--n', dest='train_count', action='store', type=str, help='Number of training images needed')
    parser.add_argument('--o', dest='out_dir', action='store', type=str, help='Output directory')
    args = parser.parse_args()
        
    # Open the video capture
    cap = cv2.VideoCapture(args.video_path)
    
    # Get number of frames and downsample 
    length = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))
    down_sample_value = int(length/int(args.train_count))
    
    # Create output path if it doesn't exist
    if not os.path.exists(args.out_dir):
        os.makedirs(args.out_dir)
    else:
        sys.exit("Output path already exists")
    
    # Iterate through the video 
    while(cap.isOpened()):      
        # Get the frame ID and the frame
        frameID = cap.get(1)
        ret, frame = cap.read()
    
        if ret == True:
            # Save every 13th frame
            if frameID % down_sample_value == 0:          
                # Get the left camera view
                print("saving frame " + str(int(np.floor(frameID/down_sample_value))))
                frame_dst = frame[:,0:frame.shape[1]/2] 
                cv2.imwrite(args.out_dir + "/frame_{:04d}".format(int(np.floor(frameID/down_sample_value))) + ".png", frame_dst)
        else:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv[1:])