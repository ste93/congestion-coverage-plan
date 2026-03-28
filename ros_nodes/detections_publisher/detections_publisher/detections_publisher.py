from rclpy.impl import rcutils_logger
from ament_index_python.packages import get_package_share_directory
import os
from static_devices_msgs.msg import DetectionsArray, SingleDetection


import rclpy
from rclpy.node import Node
from congestion_coverage_plan.detections_retriever.DetectionsRetriever import FakeDetectionsRetriever
import sys

class DetectionsPublisher(Node):

    def __init__(self, detections_topic, initial_time=0.0):
        super().__init__('detections_publisher')
        self._topic = detections_topic
        self.csv_path = os.path.join(
            get_package_share_directory('detections_publisher'),
            'resource',
            'madama3_september.csv'
        )
        self.timestamp = initial_time
        # create a fake detections retriever to read from the CSV file
        self._detections_retriever = FakeDetectionsRetriever(self.csv_path)

        # create a ros2 publisher
        
        self.publisher_ = self.create_publisher(DetectionsArray, self._topic, 10)
        self.create_timer(1, self.publish_detections)
        
    def publish_detections(self):
        # Here you would add the logic to read from the CSV and publish detections
        # For demonstration, we will just print a message
        # self.get_logger().info(f'Publishing detections from {self.csv_path} on topic {self._topic}')
        self._detections_retriever.set_timestamp(self.timestamp)
        detections = self._detections_retriever.get_current_occupancies()
        self.get_logger().info(f'timestamp: {self.timestamp}, detections: {detections}')
        self.timestamp = self.timestamp + 1
        # create the message
        msg = DetectionsArray()
        msg.detections = []
        print(detections)
        for detection in detections:
            msg.detections.append(SingleDetection(
                id=int(detection.person_id),
                x=float(detection.positionx),
                y=float(detection.positiony),
                vx=float(detection.vx),
                vy=float(detection.vy),
                ts=int(detection.timestamp)
            ))
        # publish
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    detections_topic = "static_tracks"
    initial_time = 0.0
    if "--detections-topic" in sys.argv:
        idx = sys.argv.index("--detections-topic")
        if idx + 1 < len(sys.argv):
            detections_topic = sys.argv[idx + 1]
            print(f"using topic {detections_topic}")
    if "--initial-time" in sys.argv:
        idx = sys.argv.index("--initial-time")
        if idx + 1 < len(sys.argv):
            initial_time = float(sys.argv[idx + 1])
            print(f"using initial time {initial_time}")
    if "--help" in sys.argv or "-h" in sys.argv:
        print("Usage: detections_publisher.py [--detections-topic TOPIC] [--initial_time TIME]")
        sys.exit(0)
    detections_publisher = DetectionsPublisher(detections_topic, initial_time)
    try:
        rclpy.spin(detections_publisher)
    except KeyboardInterrupt:
        pass
    detections_publisher.destroy_node()
    rclpy.shutdown()