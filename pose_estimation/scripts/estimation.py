import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import os
import sys
import errno

from common.camera import *
from common.model import *
from common.loss import *
from common.generators import ChunkedGenerator, UnchunkedGenerator
from time import time
from common.utils import deterministic_random

def estimate_pose_3D(fix):

    print('Loading dataset...')
    #dataset_path = '/home/rick/nao/src/pose_estimation/scripts/data/data_3d_' + 'custom' + '.npz'

    from common.custom_dataset import CustomDataset
    dataset = CustomDataset('/home/rick/nao/src/pose_estimation/scripts/data/data_2d_' + 'custom' + '_' + 'joints' + '.npz', remove_static_joints=fix)


    print('Preparing data...')
    for subject in dataset.subjects():
        for action in dataset[subject].keys():
            anim = dataset[subject][action]
            
            if 'positions' in anim:
                positions_3d = []
                for cam in anim['cameras']:
                    pos_3d = world_to_camera(anim['positions'], R=cam['orientation'], t=cam['translation'])
                    pos_3d[:, 1:] -= pos_3d[:, :1] # Remove global offset, but keep trajectory in first position
                    positions_3d.append(pos_3d)
                anim['positions_3d'] = positions_3d

    print('Loading 2D detections...')
    keypoints = np.load('/home/rick/nao/src/pose_estimation/scripts/data/data_2d_' + 'custom'  + '_' + 'joints' + '.npz', allow_pickle=True)
    print(keypoints)
    keypoints_metadata = keypoints['metadata'].item()
    keypoints_symmetry = keypoints_metadata['keypoints_symmetry']
    kps_left, kps_right = list(keypoints_symmetry[0]), list(keypoints_symmetry[1])
    joints_left, joints_right = list(dataset.skeleton().joints_left()), list(dataset.skeleton().joints_right())
    keypoints = keypoints['positions_2d'].item()

    for subject in dataset.subjects():
        assert subject in keypoints, 'Subject {} is missing from the 2D detections dataset'.format(subject)
        for action in dataset[subject].keys():
            assert action in keypoints[subject], 'Action {} of subject {} is missing from the 2D detections dataset'.format(action, subject)
            if 'positions_3d' not in dataset[subject][action]:
                continue
                
            for cam_idx in range(len(keypoints[subject][action])):
                
                # We check for >= instead of == because some videos in H3.6M contain extra frames
                mocap_length = dataset[subject][action]['positions_3d'][cam_idx].shape[0]
                assert keypoints[subject][action][cam_idx].shape[0] >= mocap_length
                
                if keypoints[subject][action][cam_idx].shape[0] > mocap_length:
                    # Shorten sequence
                    keypoints[subject][action][cam_idx] = keypoints[subject][action][cam_idx][:mocap_length]

            assert len(keypoints[subject][action]) == len(dataset[subject][action]['positions_3d'])
            
    for subject in keypoints.keys():
        for action in keypoints[subject]:
            for cam_idx, kps in enumerate(keypoints[subject][action]):
                # Normalize camera frame
                cam = dataset.cameras()[subject][cam_idx]
                kps[..., :2] = normalize_screen_coordinates(kps[..., :2], w=cam['res_w'], h=cam['res_h'])
                keypoints[subject][action][cam_idx] = kps

    subjects_test = ["joints"]

    def fetch(subjects, action_filter=None, subset=1, parse_3d_poses=True):
        out_poses_3d = []
        out_poses_2d = []
        out_camera_params = []
        
        print(keypoints.keys())
        
        for subject in subjects:
            print(subject)
            for action in keypoints[subject].keys():
                if action_filter is not None:
                    found = False
                    for a in action_filter:
                        if action.startswith(a):
                            found = True
                            break
                    if not found:
                        continue
                    
                poses_2d = keypoints[subject][action]
                for i in range(len(poses_2d)): # Iterate across cameras
                    out_poses_2d.append(poses_2d[i])
                    
                if subject in dataset.cameras():
                    cams = dataset.cameras()[subject]
                    assert len(cams) == len(poses_2d), 'Camera count mismatch'
                    for cam in cams:
                        if 'intrinsic' in cam:
                            out_camera_params.append(cam['intrinsic'])
                    
                if parse_3d_poses and 'positions_3d' in dataset[subject][action]:
                    poses_3d = dataset[subject][action]['positions_3d']
                    assert len(poses_3d) == len(poses_2d), 'Camera count mismatch'
                    for i in range(len(poses_3d)): # Iterate across cameras
                        out_poses_3d.append(poses_3d[i])
        
        if len(out_camera_params) == 0:
            out_camera_params = None
        if len(out_poses_3d) == 0:
            out_poses_3d = None
        
        stride = 0
        if subset < 1:
            for i in range(len(out_poses_2d)):
                n_frames = int(round(len(out_poses_2d[i])//stride * subset)*stride)
                start = deterministic_random(0, len(out_poses_2d[i]) - n_frames + 1, str(len(out_poses_2d[i])))
                out_poses_2d[i] = out_poses_2d[i][start:start+n_frames:stride]
                if out_poses_3d is not None:
                    out_poses_3d[i] = out_poses_3d[i][start:start+n_frames:stride]
        elif stride > 1:
            # Downsample as requested
            for i in range(len(out_poses_2d)):
                out_poses_2d[i] = out_poses_2d[i][::stride]
                if out_poses_3d is not None:
                    out_poses_3d[i] = out_poses_3d[i][::stride]
        

        return out_camera_params, out_poses_3d, out_poses_2d

    action_filter = None
        
    cameras_valid, poses_valid, poses_valid_2d = fetch(subjects_test, action_filter)

    architecture = "3,3,3,3,3"

    filter_widths = [int(x) for x in architecture.split(',')]

    # Use optimized model for single-frame predictions
    model_pos_train = TemporalModelOptimized1f(poses_valid_2d[0].shape[-2], poses_valid_2d[0].shape[-1], dataset.skeleton().num_joints(),
                                    filter_widths=filter_widths, causal=True, dropout=0.25, channels=1024)

        
    model_pos = TemporalModel(poses_valid_2d[0].shape[-2], poses_valid_2d[0].shape[-1], dataset.skeleton().num_joints(),
                                filter_widths=filter_widths, causal=True, dropout=0.25, channels=1024,
                                dense=None)

    receptive_field = model_pos.receptive_field()
    print('INFO: Receptive field: {} frames'.format(receptive_field))
    pad = (receptive_field - 1) // 2 # Padding on each side

    causal_shift = 0

    model_params = 0
    for parameter in model_pos.parameters():
        model_params += parameter.numel()
    print('INFO: Trainable parameter count:', model_params)

    if torch.cuda.is_available():
        model_pos = model_pos.cuda()
        model_pos_train = model_pos_train.cuda()
        
    chk_filename = "/home/rick/nao/src/pose_estimation/scripts/checkpoint/pretrained_h36m_detectron_coco.bin"
    print('Loading checkpoint', chk_filename)
    checkpoint = torch.load(chk_filename, map_location=lambda storage, loc: storage)
    print('This model was trained for {} epochs'.format(checkpoint['epoch']))
    model_pos_train.load_state_dict(checkpoint['model_pos'])
    model_pos.load_state_dict(checkpoint['model_pos'])

    if "pretrained_h36m_detectron_coco.bin" and 'model_traj' in checkpoint:
        # Load trajectory model if it contained in the checkpoint (e.g. for inference in the wild)
        model_traj = TemporalModel(poses_valid_2d[0].shape[-2], poses_valid_2d[0].shape[-1], 1,
                            filter_widths=filter_widths, causal=True, dropout=0.25, channels=1024,
                            dense=True)
        if torch.cuda.is_available():
            model_traj = model_traj.cuda()
        model_traj.load_state_dict(checkpoint['model_traj'])
    else:
        model_traj = None
            
        
    test_generator = UnchunkedGenerator(cameras_valid, poses_valid, poses_valid_2d,
                                        pad=pad, causal_shift=causal_shift, augment=False,
                                        kps_left=kps_left, kps_right=kps_right, joints_left=joints_left, joints_right=joints_right)
    print('INFO: Testing on {} frames'.format(test_generator.num_frames()))

    def evaluate(test_generator, action=None, return_predictions=False, use_trajectory_model=False):
        epoch_loss_3d_pos = 0
        epoch_loss_3d_pos_procrustes = 0
        epoch_loss_3d_pos_scale = 0
        epoch_loss_3d_vel = 0
        with torch.no_grad():
            if not use_trajectory_model:
                model_pos.eval()
            else:
                model_traj.eval()
            N = 0
            for _, batch, batch_2d in test_generator.next_epoch():
                inputs_2d = torch.from_numpy(batch_2d.astype('float32'))
                if torch.cuda.is_available():
                    inputs_2d = inputs_2d.cuda()

                # Positional model
                if not use_trajectory_model:
                    predicted_3d_pos = model_pos(inputs_2d)
                else:
                    predicted_3d_pos = model_traj(inputs_2d)

                # Test-time augmentation (if enabled)
                if test_generator.augment_enabled():
                    # Undo flipping and take average with non-flipped version
                    predicted_3d_pos[1, :, :, 0] *= -1
                    if not use_trajectory_model:
                        predicted_3d_pos[1, :, joints_left + joints_right] = predicted_3d_pos[1, :, joints_right + joints_left]
                    predicted_3d_pos = torch.mean(predicted_3d_pos, dim=0, keepdim=True)
                    
                if return_predictions:
                    return predicted_3d_pos.squeeze(0).cpu().numpy()
                    
                inputs_3d = torch.from_numpy(batch.astype('float32'))
                if torch.cuda.is_available():
                    inputs_3d = inputs_3d.cuda()
                inputs_3d[:, :, 0] = 0    
                if test_generator.augment_enabled():
                    inputs_3d = inputs_3d[:1]

                error = mpjpe(predicted_3d_pos, inputs_3d)
                epoch_loss_3d_pos_scale += inputs_3d.shape[0]*inputs_3d.shape[1] * n_mpjpe(predicted_3d_pos, inputs_3d).item()

                epoch_loss_3d_pos += inputs_3d.shape[0]*inputs_3d.shape[1] * error.item()
                N += inputs_3d.shape[0] * inputs_3d.shape[1]
                
                inputs = inputs_3d.cpu().numpy().reshape(-1, inputs_3d.shape[-2], inputs_3d.shape[-1])
                predicted_3d_pos = predicted_3d_pos.cpu().numpy().reshape(-1, inputs_3d.shape[-2], inputs_3d.shape[-1])

                epoch_loss_3d_pos_procrustes += inputs_3d.shape[0]*inputs_3d.shape[1] * p_mpjpe(predicted_3d_pos, inputs)

                # Compute velocity error
                epoch_loss_3d_vel += inputs_3d.shape[0]*inputs_3d.shape[1] * mean_velocity_error(predicted_3d_pos, inputs)
                
        if action is None:
            print('----------')
        else:
            print('----'+action+'----')
        e1 = (epoch_loss_3d_pos / N)*1000
        e2 = (epoch_loss_3d_pos_procrustes / N)*1000
        e3 = (epoch_loss_3d_pos_scale / N)*1000
        ev = (epoch_loss_3d_vel / N)*1000
        print('Test time augmentation:', test_generator.augment_enabled())
        print('Protocol #1 Error (MPJPE):', e1, 'mm')
        print('Protocol #2 Error (P-MPJPE):', e2, 'mm')
        print('Protocol #3 Error (N-MPJPE):', e3, 'mm')
        print('Velocity Error (MPJVE):', ev, 'mm')
        print('----------')

        return e1, e2, e3, ev

            # Evaluate
    print('Rendering...')

    input_keypoints = keypoints["joints"]["custom"][0].copy()
    ground_truth = None
    if "joints" in dataset.subjects() and "custom" in dataset["joints"]:
        if 'positions_3d' in dataset["joints"]["custom"]:
            ground_truth = dataset["joints"]["custom"]['positions_3d'][0].copy()
    if ground_truth is None:
        print('INFO: this action is unlabeled. Ground truth will not be rendered.')
        
    gen = UnchunkedGenerator(None, None, [input_keypoints],
                                pad=pad, causal_shift=causal_shift, augment=None,
                                kps_left=kps_left, kps_right=kps_right, joints_left=joints_left, joints_right=joints_right)
    prediction = evaluate(gen, return_predictions=True)
    if model_traj is not None and ground_truth is None:
        prediction_traj = evaluate(gen, return_predictions=True, use_trajectory_model=True)
        prediction += prediction_traj

    print("Done")

    return prediction[49]