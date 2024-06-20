from typing import Tuple
#Import math Library
import math 

FOCAL_X = 333.4101186407986
FOCAL_Y = 333.6774109483744
C_X = 324.6950963207407
C_Y = 224.5743511258171
BIAS_CAMERA = 0.175 # y offset camera
Z_CAM = 0.11 #m
DUCKIE_HEIGHT = 0.038 #m
LEMON_HEIGHT = 0.062 #m
ORANGE_HEIGHT = 0.068 #m
PITCH = -10 #deg


def DT_TOKEN() -> str:
    # TODO: change this to your duckietown token
    dt_token = "dt1-3nT7FDbT7NLPrXykNJmqqhCXsvh63uUPdmEo8kFqKrho1Cp-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbffv9cvFiBNwtbG52K3PdVZzNNY4rgMJkfp"
    return dt_token


def MODEL_NAME() -> str:
    # TODO: change this to your model's name that you used to upload it on google colab.
    # if you didn't change it, it should be "yolov5n"
    return "yolov5n"


def NUMBER_FRAMES_SKIPPED() -> int:
    # TODO: change this number to drop more frames
    # (must be a positive integer)
    return 0


def filter_by_classes(pred_class: int) -> bool:
    """
    Remember the class IDs:

        | Object       | ID    |
        | ---          | ---   |
        | apple        | 0     |
        | lemon        | 1     |
        | orange       | 2     |
        | pear         | 3     |
        | strawberry   | 4     |
        | duckie       | 5     |
        | cone         | 6     |
        | truck        | 7     |
        | bus          | 8     |

    Args:
        pred_class: the class of a prediction
    """
    # Right now, this returns True for every object's class
    # TODO: Change this to only return True for duckies!
    # In other words, returning False means that this prediction is ignored.

    # if pred_class == 0:
    #     return True
    # else:  
    #     return False
    #return pred_class == 1 or pred_class == 2 or pred_class == 5
    return True

def filter_by_scores(score: float) -> bool:
    """
    Args:
        score: the confidence score of a prediction
    """
    # Right now, this returns True for every object's confidence
    # TODO: Change this to filter the scores, or not at all
    # (returning True for all of them might be the right thing to do!)
    if score >= 0.50:
        return True
    else:
        return False


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    """
    Args:
        bbox: is the bounding box of a prediction, in xyxy format
                This means the shape of bbox is (leftmost x pixel, topmost y, rightmost x, bottommost y)
    """
    # TODO: Like in the other cases, return False if the bbox should not be considered.
    return True

def depth_estimation(bbox: Tuple[int, int, int, int], pred_class: int):
    """
    Args:
        bbox: is the bounding box of a prediction, in xyxy format
                This means the shape of bbox is (leftmost x pixel, topmost y, rightmost x, bottommost y)
    """
    if pred_class == 0:
        height = DUCKIE_HEIGHT
    elif pred_class == 2:
        height = ORANGE_HEIGHT
    elif pred_class == 1:
        height = LEMON_HEIGHT
    else:
        height = DUCKIE_HEIGHT
        print("predicted class not known, default to duckie height\n")

    cam_ref_z = (height*FOCAL_Y)/(bbox[1]-bbox[3])
    bb_xcenter = (bbox[0]+bbox[2])/2
    ref_y = -cam_ref_z*(bb_xcenter-C_X)/FOCAL_X
    ref_x = -cam_ref_z
    # convert to polar
    dist = (ref_x**2 + ref_y**2)**(0.5)
    angle = math.atan2(ref_y,ref_x)

    return dist, angle