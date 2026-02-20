import matplotlib
from congestion_coverage_plan.detections_retriever.DetectionsRetriever import DetectionsRetriever
import time
from rclpy.impl import rcutils_logger
from ament_index_python.packages import get_package_share_directory
import os

from src.congestion_coverage_plan_museum.utils import dataset_utils
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from static_devices_msgs.msg import DetectionsArray, SingleDetection
import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from congestion_coverage_plan.mdp.MDP import MDP, State
from congestion_coverage_plan.map_utils.OccupancyMap import OccupancyMap
from congestion_coverage_plan.cliff_predictor.PredictorCreator import create_madama_cliff_predictor
from congestion_coverage_plan.bt_utils.BTWriter import BTWriter
from congestion_coverage_plan.detections_retriever.DetectionsRetriever import DetectionsRetriever
from congestion_coverage_plan.solver.LrtdpTvmaAlgorithm import LrtdpTvmaAlgorithm
import sys

class DetectionsVisualizer(Node):

    def __init__(self, detections_topic, data_folder="./"):
        super().__init__('detections_visualizer')
        self._detections_retriever = DetectionsRetriever(self, detections_topic, queue_size=10)
        self.img_path = os.path.join(
            get_package_share_directory('detections_visualizer'),
            'resource',
            'madama3.jpg'
        )
        self._cliff_map_file = os.path.join(
            get_package_share_directory('detections_visualizer'),
            'config',
            'map_madama3_september.csv'
        )
        self._cliff_predictor = create_madama_cliff_predictor(data_folder=data_folder)

        plt.ion()
        # set background image
        self.fig_size = [-21.2,36.4, -53.4, 9.2 ]
        self.fig, self.ax = plt.subplots()
        self.img = plt.imread(str(self.img_path))
        self.ax.imshow(self.img, cmap='gray', vmin=0, vmax=255, extent=self.fig_size)
    
        # create a graph to be updated dynamically with the detections
        # self.scatter = self.ax.scatter([], [])
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_title('Detections')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        # plt.show(block=False)  # <-- Add this line
        self.create_timer(1, self.plot_detections)

    def get_tracks(self):
        self.human_traj_data = self._detections_retriever.get_detections()
        tracks = {}
        print("observed_tracklet_length:", self._cliff_predictor.observed_tracklet_length)
        for id in self.human_traj_data.keys():

            human_traj_data_by_person_id = self.human_traj_data[id]
            # convert from list of Detection to numpy array
            print("Processing id:", id)
            # print("human_traj_data_by_person_id:", human_traj_data_by_person_id)

            datatype = np.dtype([('timestamp', 'f8'), ('x', 'f8'), ('y', 'f8'), ('velocity', 'f8'), ('motion_angle', 'f8')])
            # local_trajectory = np.rec.array([])
            local_trajectory = []
            for detection in human_traj_data_by_person_id:
                # print("detection:", detection.timestamp, detection.positionx, detection.positiony, detection.vx, detection.vy)
                local_trajectory.append((detection.timestamp, detection.positionx, detection.positiony, detection.velocity, detection.motion_angle))
            # print ("local_trajectory:", local_trajectory)
            human_traj_array = np.array(local_trajectory, dtype=datatype)
            # print("human_traj_array:", human_traj_array)
            # filter the trajectory to be only before the time
            # track_before_now = human_traj_array[human_traj_array['timestamp'] <= time]
            # track_before_now = human_traj_array[human_traj_array[:, 0] <= time]
            # track_filtered = track_before_now[-(self._cliff_predictor.observed_tracklet_length + 1):]
            if len(human_traj_array) >= self._cliff_predictor.observed_tracklet_length :
                tracks[id] = human_traj_array[-(self._cliff_predictor.observed_tracklet_length + 1 ):]
        for track_id in tracks.keys():
            print("track_id:", track_id, "track:", tracks[track_id])
            print("============================= track length:", len(tracks[track_id]))
            
        return tracks
            

    def plot_detections(self):

        
        # detections_local = self._detections_retriever.get_detections()
        # print(detections_local)
        tracks = self.get_tracks()
        all_predictions = self._cliff_predictor.predict_positions(tracks)
        # print("all_predictions:", len(all_predictions))
        self.ax.cla()
        self.ax.imshow(self.img, cmap='gray', vmin=0, vmax=255, extent=self.fig_size)
        for det_id in tracks.keys():
            for det in tracks[det_id]:
                self.ax.plot(det['x'], det['y'], 'o', color='red')
        for predicted_people in all_predictions:
            for predicted_traj in predicted_people:
                # if len(predicted_traj) < self._cliff_predictor.planning_horizon:
                #     print("predicted_traj too short:", len(predicted_traj))
                #     continue
                for traj_point in predicted_traj:
                    self.ax.plot(traj_point[1], traj_point[2], '.', color='blue', alpha=0.3)
                # final_pose = predicted_traj[self._cliff_predictor.planning_horizon - 1]
                # self.ax.scatter(final_pose[1], final_pose[2], marker='D', alpha=1, color="b", s=100, label="Predicted position")

        plt.draw()
        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    detections_topic = "static_tracks"
    if "--data_folder" in sys.argv:
        idx = sys.argv.index("--data_folder")
        if idx + 1 < len(sys.argv):
            data_folder = sys.argv[idx + 1]
            print(f"using data folder {data_folder}")
    detections_visualizer = DetectionsVisualizer(detections_topic, data_folder=data_folder)
    try:
        rclpy.spin(detections_visualizer)
    except KeyboardInterrupt:
        pass
    detections_visualizer.destroy_node()
    rclpy.shutdown()