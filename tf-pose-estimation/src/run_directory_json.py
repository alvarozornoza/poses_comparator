import argparse
import logging
import time
import glob
import ast
import os
import dill

import common
import cv2
import numpy as np
from estimator import TfPoseEstimator
from networks import get_graph_path, model_wh

#from lifting.prob_model import Prob3dPose
#from lifting.draw import plot_pose

from exportData import produceSkeletonData
import os
import json


logger = logging.getLogger('TfPoseEstimator')
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='tf-pose-estimation run by folder')
    parser.add_argument('--folder', type=str, default='./images/')
    parser.add_argument('--resolution', type=str, default='432x368', help='network input resolution. default=432x368')
    parser.add_argument('--model', type=str, default='mobilenet_thin', help='cmu / mobilenet_thin')
    parser.add_argument('--scales', type=str, default='[None]', help='for multiple scales, eg. [1.0, (1.1, 0.05)]')
    parser.add_argument('--resize', type=str, default='0x0',
                        help='if provided, resize images before they are processed. default=0x0, Recommends : 432x368 or 656x368 or 1312x736 ')
    parser.add_argument('--resize-out-ratio', type=float, default=4.0,
                        help='if provided, resize heatmaps before they are post-processed. default=1.0')

    args = parser.parse_args()
    scales = ast.literal_eval(args.scales)

    w, h = model_wh(args.resolution)
    e = TfPoseEstimator(get_graph_path(args.model), target_size=(w, h))

    files_grabbed = glob.glob(os.path.join(args.folder + "rgb/", '*.png'))
    all_humans = dict()

    file = open(args.folder + "json/openpose.txt","w") 
    skeletonsData = []
    for i, picture in enumerate(files_grabbed):
        # estimate human poses from a single image !
        ##image = common.read_imgfile(picture, None, None)
        image = common.read_imgfile(args.folder + "rgb/" + str(i) + ".png")
        t = time.time()
        #humans = e.inference(image, scales=scales)
        ##humans = e.inference(image)
        humans = e.inference(image, resize_to_default=(w > 0 and h > 0), upsample_size=args.resize_out_ratio)

        elapsed = time.time() - t

        logger.info('inference image #%d: %s in %.4f seconds.' % (i, picture, elapsed))

        image = TfPoseEstimator.draw_humans(image, humans, imgcopy=False)
        cv2.imshow('tf-pose-estimation result', image)

        skeleton = produceSkeletonData(i,humans)
        skeletonsData.append(skeleton)
        #print(json.dumps(produceSkeletonData(i,humans),indent=4, separators=(',', ': ')))


        cv2.waitKey(5)

        all_humans[picture.replace(args.folder, '')] = humans
    
    file.write(json.dumps(skeletonsData, indent=4, separators=(',', ': ')))
    file.close()

   ## with open(os.path.join(args.folder, 'pose.dil'), 'wb') as f:
    ##    dill.dump(all_humans, f, protocol=dill.HIGHEST_PROTOCOL)
