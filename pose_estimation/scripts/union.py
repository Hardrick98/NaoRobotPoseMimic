#! /usr/bin/env python3

"""Perform inference on a single video or all videos with a certain extension
(e.g., .mp4) in a folder.
"""
import os
import detectron2
from detectron2.utils.logger import setup_logger
from detectron2.config import get_cfg
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
import cv2
from data_utils import suggest_metadata
import subprocess as sp
import numpy as np
import time
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from estimation import estimate_pose_3D


output_prefix_2d = '/home/rick/nao/src/pose_estimation/scripts/data/data_2d_custom_'

def decode(bb, kp, metadata):
    # Latin1 encoding because Detectron runs on Python 2.7
    
    metadata = metadata
    results_bb = []
    results_kp = []
    for i in range(len(bb)):
        if len(bb[i][1]) == 0 or len(kp[i][1]) == 0:
            # No bbox/keypoints detected for this frame -> will be interpolated
            results_bb.append(np.full(4, np.nan, dtype=np.float32)) # 4 bounding box coordinates
            results_kp.append(np.full((17, 4), np.nan, dtype=np.float32)) # 17 COCO keypoints
            continue
        best_match = np.argmax(bb[i][1][:, 4])
        best_bb = bb[i][1][best_match, :4]
        best_kp = kp[i][1][best_match].T.copy()
        results_bb.append(best_bb)
        results_kp.append(best_kp)
        
    bb = np.array(results_bb, dtype=np.float32)
    kp = np.array(results_kp, dtype=np.float32)
    kp = kp[:, :, :2] # Extract (x, y)
    
    # Fix missing bboxes/keypoints by linear interpolation
    mask = ~np.isnan(bb[:, 0])
    indices = np.arange(len(bb))
    for i in range(4):
        bb[:, i] = np.interp(indices, indices[mask], bb[mask, i])
    for i in range(17):
        for j in range(2):
            kp[:, i, j] = np.interp(indices, indices[mask], kp[mask, i, j])
    
    print('{} total frames processed'.format(len(bb)))
    print('{} frames were interpolated'.format(np.sum(~mask)))
    print('----------')
    
    return [{
        'start_frame': 0, # Inclusive
        'end_frame': len(kp), # Exclusive
        'bounding_boxes': bb,
        'keypoints': kp,
    }], metadata

fix = True

cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-Keypoints/keypoint_rcnn_R_101_FPN_3x.yaml"))
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7
cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-Keypoints/keypoint_rcnn_R_101_FPN_3x.yaml")
predictor = DefaultPredictor(cfg)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

boxes = []
segments = []
keypoints = []

count = 0


while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    t = time.time()
    outputs = predictor(frame)['instances'].to('cpu')
    
    has_bbox = False
    if outputs.has('pred_boxes'):
        bbox_tensor = outputs.pred_boxes.tensor.numpy()
        if len(bbox_tensor) > 0:
            has_bbox = True
            scores = outputs.scores.numpy()[:, None]
            bbox_tensor = np.concatenate((bbox_tensor, scores), axis=1)
    if has_bbox:
        kps = outputs.pred_keypoints.numpy()
        kps_xy = kps[:, :, :2]
        kps_prob = kps[:, :, 2:3]
        kps_logit = np.zeros_like(kps_prob) # Dummy
        kps = np.concatenate((kps_xy, kps_logit, kps_prob), axis=2)
        kps = kps.transpose(0, 2, 1)
    else:
        kps = []
        bbox_tensor = []
    

    # Mimic Detectron1 format
    cls_boxes = [[], bbox_tensor]
    cls_keyps = [[], kps]

    boxes.append(cls_boxes)
    segments.append(None)
    keypoints.append(cls_keyps)

    bb = [cls_boxes]
    kp = [cls_keyps]
    
    results_bb = []
    results_kp = []

    for i in range(len(bb)):
        if len(bb[i][1]) == 0 or len(kp[i][1]) == 0:
            # No bbox/keypoints detected for this frame -> will be interpolated
            results_bb.append(np.full(4, np.nan, dtype=np.float32)) # 4 bounding box coordinates
            results_kp.append(np.full((17, 4), np.nan, dtype=np.float32)) # 17 COCO keypoints
            continue
        best_match = np.argmax(bb[i][1][:, 4])
        best_bb = bb[i][1][best_match, :4]
        best_kp = kp[i][1][best_match].T.copy()
        results_bb.append(best_bb)
        results_kp.append(best_kp)
        
    bb = np.array(results_bb, dtype=np.float32)
    kp = np.array(results_kp, dtype=np.float32)
    kp = kp[:, :, :2] # Extract (x, y)

# Fix missing bboxes/keypoints by linear interpolation
    mask = ~np.isnan(bb[:, 0])
    indices = np.arange(len(bb))
    for i in range(4):
        bb[:, i] = np.interp(indices, indices[mask], bb[mask, i])
    for i in range(17):
        for j in range(2):
            kp[:, i, j] = np.interp(indices, indices[mask], kp[mask, i, j])
            
    for p in kp[0]:
        image = cv2.circle(frame, (int(p[0]),int(p[1])), radius=4, color=(255, 255, 255), thickness=-1)

    # Display the resulting frame
    
    if cv2.waitKey(1) == ord('q'):
        break
    
    cv2.imshow("frame", image)

    count+=1

    if count == 50:

        bb = boxes
        kp = keypoints

        metadata_original = {
        'w': frame.shape[1],
        'h': frame.shape[0],
     }


        metadata = suggest_metadata('coco')
        metadata['video_metadata'] = {}

        output = {}
        canonical_name = "joints"


        data, video_metadata = decode(bb,kp,metadata_original)
        output[canonical_name] = {}
        output[canonical_name]['custom'] = [data[0]['keypoints'].astype('float32')]
        metadata['video_metadata'][canonical_name] = video_metadata

        print('Saving...')
        np.savez_compressed(output_prefix_2d + "joints", positions_2d=output, metadata=metadata)
        print('Done.')

        file_path = "/home/rick/nao/src/pose_estimation/scripts/data/data_2d_custom_joints.npz"

        count=0

        boxes = []
        keypoints = []
        segments = []



       
        prediction = estimate_pose_3D(fix)

        os.remove(file_path)
        print(f"File {file_path} removed successfully.")
        
        try:


            rospy.init_node('pose3d_publisher', anonymous=True)

            
            pub = rospy.Publisher('lhand_pose_topic', Float32MultiArray, queue_size=10)
            pub2 = rospy.Publisher('rhand_pose_topic', Float32MultiArray, queue_size=10)

           
                
            lhand_msg = Float32MultiArray(data=prediction[13].tolist())  # Convert to Python list
            pub.publish(lhand_msg)

            rhand_msg = Float32MultiArray(data=prediction[16].tolist()) 
            pub2.publish(lhand_msg)

        except rospy.ROSInterruptException:
            pass

        fix = False



        




cap.release()
cv2.destroyAllWindows()















