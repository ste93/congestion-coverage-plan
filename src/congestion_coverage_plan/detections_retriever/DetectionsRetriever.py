# topic subrscriber for retrieving detections from a ROS topic
# it keeps the latest detection messages in a member variable

from threading import Lock
from src.congestion_coverage_plan_museum.utils import dataset_utils
from static_devices_msgs.msg import DetectionsArray, SingleDetection
from matplotlib import pyplot as plt
from congestion_coverage_plan.utils.dataset_utils import read_human_traj_data_from_file

class Detection:
    def __init__(self, person_id, positionx, positiony, timestamp, velocity, motion_angle):
        self.person_id = person_id
        self.positionx = positionx
        self.positiony = positiony
        self.timestamp = timestamp
        self.velocity = velocity
        self.motion_angle = motion_angle


class DetectionsRetriever:
    def __init__(self, node=None, topic_name="static_tracks", queue_size=10):
        """
        If `node` (rclpy.node.Node) is provided, subscription is created immediately.
        Otherwise call start(node=...) or start_background_node() to run standalone.
        """
        self._node = node
        self._topic_name = topic_name
        self._lock = Lock()
        self._detections = {}
        self._current_occupancies = {}
        self._queue_size = queue_size
        self._subscriber = None

        self.start_with_node(node)


        


    def _callback(self, msg):
        detections_local = {}
        current_occupancies_local = []
        for detection in msg.detections:
            detection: SingleDetection
            velocity, motion_angle = dataset_utils.convert_vx_vy_to_velocity_motion_angle(detection.vx, detection.vy)
            detection_obj = Detection(
                person_id=detection.id,
                positionx=detection.x,
                positiony=detection.y,
                timestamp=detection.ts,
                velocity=velocity,
                motion_angle=motion_angle
            )
            existing = self._detections.get(detection.id, [])
            # keep newest first, limit to queue_size
            combined = [detection_obj] + existing[:5] if existing else [detection_obj]
            detections_local[detection.id] = sorted(combined, key=lambda x: x.timestamp, reverse=True)
            
            current_occupancies_local.append(Detection(
                person_id=detection.id,
                positionx=detection.x,
                positiony=detection.y,
                timestamp=detection.ts,
                velocity=velocity,
                motion_angle=motion_angle
            ))

        with self._lock:
            self._detections = detections_local
            self._current_occupancies = current_occupancies_local

    def check_for_stale_detections(self):
        if len(self._detections) > 0:
            latest_timestamp = max(detections[0].timestamp for detections in self._detections.values())
            if self._node is not None:
                current_time = self._node.get_clock().now().to_msg().sec
                if current_time - latest_timestamp > 5:  # 5 seconds threshold for staleness
                    with self._lock:
                        self._detections = {}
                        self._current_occupancies = {}

    def get_detections(self):
        self.check_for_stale_detections()
        with self._lock:
            return self._detections

    def get_current_occupancies(self):
        self.check_for_stale_detections()
        with self._lock:
            return self._current_occupancies
        
    def start_with_node(self, node):
        """Attach to an existing node (create subscription if needed)."""
        self._node = node
        if self._subscriber is None:
            self._subscriber = self._node.create_subscription(DetectionsArray, self._topic_name, self._callback, self._queue_size)
        return True

class FakeDetectionsRetriever:
    def __init__(self, dataset_filename, queue_size=5, timestamp = 0.0):
        self._lock = Lock()
        self._detections = {}
        self._current_occupancies = {}
        self._queue_size = queue_size
        self._dataset_filename = dataset_filename
        self.human_traj_data = None
        self.load_dataset()
        self.timestamp = timestamp

    def load_dataset(self):
        self.human_traj_data = read_human_traj_data_from_file(self._dataset_filename)


    def set_timestamp(self, timestamp):
        self.timestamp = timestamp


    def get_detections(self):
        human_traj_data_by_time = self.human_traj_data.loc[abs(self.human_traj_data['time'] - self.timestamp) < 1 ]
        people_ids = list(human_traj_data_by_time.person_id.unique())
        tracks = {}
        self.people_trajectories = {}

        for id in people_ids:
            human_traj_data_by_person_id = self.human_traj_data.loc[self.human_traj_data['person_id'] == id]
            human_traj_array = human_traj_data_by_person_id[["time", "x", "y", "vx", "vy"]].to_numpy()
            print("human_traj_array:", human_traj_array)
            track_before_now = human_traj_array[human_traj_array[:, 0] <= self.timestamp]
            track_filtered = track_before_now[-(self._queue_size + 1):]
            if len(track_filtered) >= self._queue_size:
                tracks[id] = track_filtered

        self.people_trajectories = tracks
        for track in tracks:
            detections_list = []
            for point in tracks[track]:
                velocity, motion_angle = dataset_utils.convert_vx_vy_to_velocity_motion_angle(point[3], point[4])
                detection_obj = Detection(
                    person_id=0,
                    positionx=point[1],
                    positiony=point[2],
                    timestamp=point[0],
                    velocity=velocity,
                    motion_angle=motion_angle
                )
                detections_list.append(detection_obj)
            self._detections[track] = sorted(detections_list, key=lambda x: x.timestamp, reverse=True)
        return self._detections

    

    def get_current_occupancies(self):
        human_traj_data_by_time = self.human_traj_data.loc[abs(self.human_traj_data['time'] - self.timestamp) < 1 ]
        people_ids = list(human_traj_data_by_time.person_id.unique())
        current_occupancies_local = []
        for id in people_ids:
            human_traj_data_by_person_id = self.human_traj_data.loc[self.human_traj_data['person_id'] == id]
            human_traj_array = human_traj_data_by_person_id[["time", "x", "y", "vx", "vy"]].to_numpy()
            track_before_now = human_traj_array[human_traj_array[:, 0] <= self.timestamp]
            if len(track_before_now) > 0:
                last_position = track_before_now[-1]
                velocity, motion_angle = dataset_utils.convert_vx_vy_to_velocity_motion_angle(last_position[3], last_position[4])
                current_occupancies_local.append(Detection(
                    person_id=id,
                    positionx=last_position[1],
                    positiony=last_position[2],
                    timestamp=self.timestamp,
                    velocity=velocity,
                    motion_angle=motion_angle
                ))
        return current_occupancies_local

