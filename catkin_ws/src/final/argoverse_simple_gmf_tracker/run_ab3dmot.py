#!/usr/bin/env python3

import time
import shutil

import argparse
import copy
import numpy as np
import os
from pathlib import Path
import pdb

from tqdm import tqdm
import uuid

from ab3dmot import AB3DMOT

import argoverse
from argoverse.data_loading.object_label_record import json_label_dict_to_obj_record
from argoverse.data_loading.simple_track_dataloader import SimpleArgoverseTrackingDataLoader
from argoverse.utils.se2 import SE2
from transform_utils import (
    yaw_to_quaternion3d, 
    se2_to_yaw, 
    get_B_SE2_A,
    rotmat2d
)
from json_utils import read_json_file, save_json_dict

from argoverse.evaluation.competition_util import generate_tracking_zip
from argoverse.evaluation.eval_tracking import eval_tracks

from argoverse.map_representation.map_api import ArgoverseMap


def check_mkdir(dirpath):
    """ """
    if not Path(dirpath).exists():
        os.makedirs(dirpath, exist_ok=True)



class UUIDGeneration():
    def __init__(self):
        self.mapping = {}
    def get_uuid(self,seed):
        if seed not in self.mapping:
            self.mapping[seed] = uuid.uuid4().hex 
        return self.mapping[seed]
uuid_gen = UUIDGeneration()


def yaw_from_bbox_corners(det_corners: np.ndarray) -> float:
    """
    Use basic trigonometry on cuboid to get orientation angle.

        Args:
        -   det_corners: corners of bounding box

        Returns:
        -   yaw
    """
    p1 = det_corners[1]
    p5 = det_corners[5]
    dy = p1[1] - p5[1]
    dx = p1[0] - p5[0]
    # the orientation angle of the car
    yaw = np.arctan2(dy, dx)
    return yaw


def run_ab3dmot(

    classname: str,
    pose_dir: str,
    dets_dump_dir: str,
    tracks_dump_dir: str,
    max_age: int = 3,
    min_hits: int = 1,
    min_conf: float = 0.3,
    match_algorithm: str = 'h',
    match_threshold: float = 4,
    match_distance: float = 'iou',
    p: np.ndarray = np.eye(10),
    thr_estimate: float = 0.8,
    thr_prune: float = 0.1,
    ps: float = 0.9

) -> None:
    """
    #path to argoverse tracking dataset test set, we will add our predicted labels into per_sweep_annotations_amodal/ 
    #inside this folder

    Filtering occurs in the city frame, not the egovehicle frame.

        Args:
        -   classname: string, either 'VEHICLE' or 'PEDESTRIAN'
        -   pose_dir: string
        -   dets_dump_dir: string
        -   tracks_dump_dir: string
        -   max_age: integer
        -   min_hits: integer

        Returns:
        -   None
    """
    dl = SimpleArgoverseTrackingDataLoader(data_dir=pose_dir, labels_dir=dets_dump_dir)
    
    am = ArgoverseMap()

    for log_id in tqdm(dl.sdb.get_valid_logs()):

        print(log_id)

        city_name = dl.get_city_name(log_id)

        labels_folder = dets_dump_dir + "/" + log_id + "/per_sweep_annotations_amodal/"
        lis = os.listdir(labels_folder)
        lidar_timestamps = [ int(file.split(".")[0].split("_")[-1]) for file in lis]
        lidar_timestamps.sort()
        previous_frame_bbox = []

        ab3dmot = AB3DMOT(thr_estimate=thr_estimate, thr_prune=thr_prune, ps=ps)
        
        print(labels_folder)
        tracked_labels_copy = []
        
        for j, current_lidar_timestamp in enumerate(lidar_timestamps):

            dets = dl.get_labels_at_lidar_timestamp(log_id, current_lidar_timestamp)
            
            dets_copy = dets
            transforms = []
            
            city_SE3_egovehicle = dl.get_city_to_egovehicle_se3(log_id, current_lidar_timestamp)
            egovehicle_SE3_city = city_SE3_egovehicle.inverse()
            transformed_labels = []
            conf = []

            for l_idx, l in enumerate(dets):

                if l['label_class'] != classname:
                    # will revisit in other tracking pass
                    continue
                if l["score"] < min_conf:
                    # print('Skipping det with confidence ', l["score"])
                    continue

                det_obj = json_label_dict_to_obj_record(l)
                det_corners_egovehicle_fr = det_obj.as_3d_bbox()
                
                transforms += [city_SE3_egovehicle]
                if city_SE3_egovehicle is None:
                    print('Was None')

                # convert detection from egovehicle frame to city frame
                det_corners_city_fr = city_SE3_egovehicle.transform_point_cloud(det_corners_egovehicle_fr)
                ego_xyz = np.mean(det_corners_city_fr, axis=0)

                # Check the driveable/roi area
                da = am.remove_non_driveable_area_points(np.array([ego_xyz]), city_name=city_name)
                if len(da) == 0 and l['label_class'] == 'VEHICLE':
                    continue

                roi = am.remove_non_roi_points(np.array([ego_xyz]), city_name=city_name)
                if len(roi) == 0:
                    continue
                
                yaw = yaw_from_bbox_corners(det_corners_city_fr)
                transformed_labels += [ [ego_xyz[0], ego_xyz[1], ego_xyz[2], yaw, l["length"],l["width"],l["height"]]]
                conf += [l["score"]]

            if len(transformed_labels) > 0:
                transformed_labels = np.array(transformed_labels)
            else:
                transformed_labels = np.empty((0,7))
            
            dets_all = {
                "dets": transformed_labels,
                "info": np.zeros(transformed_labels.shape),
                "conf": conf
            }

            # perform measurement update in the city frame.
            dets_with_object_id = ab3dmot.update(
                dets_all, 
                match_distance, 
                match_threshold, 
                match_algorithm,
                p
            )

            tracked_labels = []
            for det in dets_with_object_id:
                # move city frame tracks back to ego-vehicle frame
                xyz_city = np.array([det[0].item(), det[1].item(), det[2].item()]).reshape(1,3)
                city_yaw_object = det[3]
                city_se2_object = SE2(rotation=rotmat2d(city_yaw_object), translation=xyz_city.squeeze()[:2])
                city_se2_egovehicle, city_yaw_ego = get_B_SE2_A(city_SE3_egovehicle)
                ego_se2_city = city_se2_egovehicle.inverse()
                egovehicle_se2_object = ego_se2_city.right_multiply_with_se2(city_se2_object)

                # recreate all 8 points
                # transform them
                # compute yaw from 8 points once more
                egovehicle_SE3_city = city_SE3_egovehicle.inverse()
                xyz_ego = egovehicle_SE3_city.transform_point_cloud(xyz_city).squeeze()
                # update for new yaw
                # transform all 8 points at once, then compute yaw on the fly
       
                ego_yaw_obj = se2_to_yaw(egovehicle_se2_object)
                qx,qy,qz,qw = yaw_to_quaternion3d(ego_yaw_obj)

                tracked_labels.append({
                "center": {"x": xyz_ego[0], "y": xyz_ego[1], "z": xyz_ego[2]},
                "rotation": {"x": qx , "y": qy, "z": qz , "w": qw},
                "length": det[4],
                "width": det[5],
                "height": det[6],
                "track_label_uuid": uuid_gen.get_uuid(det[7]),
                 "timestamp": current_lidar_timestamp ,
                    "label_class": classname
                })

            tracked_labels_copy = copy.deepcopy(tracked_labels)

            label_dir = os.path.join(tracks_dump_dir, log_id, "per_sweep_annotations_amodal")    
            check_mkdir(label_dir)
            json_fname = f"tracked_object_labels_{current_lidar_timestamp}.json"
            json_fpath = os.path.join(label_dir, json_fname) 

            if Path(json_fpath).exists():
                # accumulate tracks of another class together
                prev_tracked_labels = read_json_file(json_fpath)
                tracked_labels.extend(prev_tracked_labels)
            
            save_json_dict(json_fpath, tracked_labels)

if __name__ == '__main__':

    split = 'test'
    #path_dataset = "/media/sda1/argoverse-tracking"
    path_dataset = "/home/ray/self-driving-car-2021/catkin_ws/src/final"
    path_detections = f"{path_dataset}/argoverse_detections_2020/testing"
    path_data = f"{path_dataset}/argoverse-tracking/{split}"
    path_results = f"{path_dataset}/results/results_tracking_{split}_cbgs"
    
    thr_estimate = 0.5
    ps = 0.9

    min_conf = 0.3
    #match_threshold = 10
    match_distance = 'iou'


    p_pos_mult, p_vel_mult = 1, 1
    p = np.eye(10)
    p[0:7,0:7] *= p_pos_mult
    p[7:,7:] *= p_vel_mult
    
    thr_prune = 0.01
    # thr_estimate = 0.5
    # ps = 0.875

    
    match_threshold = 0.001
    min_conf = 0.3

    # Vehicle
    run_ab3dmot('VEHICLE', path_data, path_detections, path_results,
        min_conf=min_conf,
        match_algorithm ='h',
        match_threshold = match_threshold,
        match_distance = match_distance,
        p = p,
        thr_estimate = thr_estimate,
        thr_prune=thr_prune,
        ps = ps
    )

    
    match_threshold = 0.001
    min_conf = 0.26


    # Pedestrian
    run_ab3dmot('PEDESTRIAN', path_data, path_detections, path_results,
        min_conf=min_conf,
        match_algorithm ='h',
        match_threshold = match_threshold,
        match_distance = match_distance,
        p = p,
        thr_estimate = thr_estimate,
        thr_prune=thr_prune,
        ps = ps
    )


    generate_tracking_zip(f"{path_dataset}/results/results_tracking_{split}_cbgs", 
        f"{path_dataset}/results")