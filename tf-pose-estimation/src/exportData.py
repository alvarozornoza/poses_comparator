import json
import common
from common import CocoPart

def produceSkeletonData(frameNumber, humans, image):
    image_h, image_w = image.shape[:2]
    skeleton = {
        "frameNumber":frameNumber,
        "score": 0,
        "keypoints": []
    }

    for human in humans:
        for i in range(common.CocoPart.Background.value):
            if i not in human.body_parts.keys():
                continue
            else:
                body_part = human.body_parts[i]
                keypoint = {
                    "position":{
                        "x": body_part.x * image_w,
                        "y": body_part.y * image_h 
                    },
                    "partName":CocoPart(body_part.part_idx).name,
                    "score": body_part.score
                }    
                skeleton['keypoints'].append(keypoint)
                skeleton['score'] = human.score

    return skeleton
