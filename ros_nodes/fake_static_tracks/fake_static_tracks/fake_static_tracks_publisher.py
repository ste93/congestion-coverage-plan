#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node

# Messaggi che stai usando nel DetectionsRetriever
from static_devices_msgs.msg import DetectionsArray, SingleDetection
import yaml

class FakeStaticTracksPublisher(Node):
    """
    Pubblica un topic finto 'static_tracks' con DetectionsArray.
    Ogni tick genera N persone che si muovono con traiettorie semplici (circolari/lineari),
    così il DetectionsRetriever accumula una history per ogni id.
    """

    def __init__(self):
        super().__init__('fake_static_tracks_publisher')

        # Parametri configurabili da command line / launch
        self.declare_parameter('topic', 'static_tracks')
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('num_people', 5)
        self.declare_parameter('arena_scale', 1.0)   # ampiezza del movimento
        self.declare_parameter('speed', 0.5)         # "velocità" angolare/lineare
        self.declare_parameter('person_list_file', '')
        
        self.topic = self.get_parameter('topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.num_people = int(self.get_parameter('num_people').value)
        self.arena_scale = float(self.get_parameter('arena_scale').value)
        self.speed = float(self.get_parameter('speed').value)
        self.person_list_file = self.get_parameter('person_list_file').value
        self.pub = self.create_publisher(DetectionsArray, self.topic, 10)
        self.persons = {}
        self.num_points_per_person = 7
        period = 1.0 / max(rate_hz, 0.1)
        self.timer = self.create_timer(period, self._on_timer)
        self.position_count = 0
        self.t0 = self.get_clock().now()
        self.id_counter = 0
        self.get_logger().info(
            f"Publishing fake detections on topic '{self.topic}' at ~{rate_hz} Hz (num_people={self.num_people})"
        )
        if self.person_list_file != '':
            self.get_logger().info(f"Using person list from file: {self.person_list_file}")
            with open(self.person_list_file, 'r') as f:
                file_loaded = yaml.safe_load(f)
                self.persons = file_loaded['persons']
                self.num_people = len(self.persons)
                self.num_points_per_person = file_loaded['num_points_per_person']   
                print(self.persons)
        else:
            self.get_logger().info("No person list file provided")
            sys.exit(1)

    def _seconds_since_start(self) -> float:
        now = self.get_clock().now()
        dt = (now - self.t0).nanoseconds / 1e9
        return float(dt)

    def _on_timer(self):
        t = self._seconds_since_start()

        msg = DetectionsArray()

        # Se il tuo DetectionsArray ha un header, puoi impostarlo qui (dipende dal .msg):
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "map"
    
        detections = []
        # ff = -0.2
        
        for person_name in self.persons.keys():
            d = SingleDetection()
            
            # timestamp = self.get_clock().now().to_msg()
            d.ts = int(self.get_clock().now().to_msg().sec)
            d.id = int(self.persons[person_name]['id']) + self.id_counter * self.num_people
            delta_x = self.persons[person_name]["vx"] * self.position_count
            delta_y = self.persons[person_name]["vy"] * self.position_count
            d.x = float(self.persons[person_name]["start_x"]) + delta_x
            d.y = float(self.persons[person_name]["start_y"]) + delta_y
            d.vx = float(self.persons[person_name]["vx"])
            d.vy = float(self.persons[person_name]["vy"])

            # phase = (2.0 * math.pi * i) / max(self.num_people, 1)

            # # posizione: moto circolare "schiacciato"
            # x = self.arena_scale * math.cos(self.speed * t + phase)
            # y = (self.arena_scale * 0.6) * math.sin(self.speed * t + phase)

            # # velocità: derivate del moto sopra (approssimate analiticamente)
            # vx = -self.arena_scale * self.speed * math.sin(self.speed * t + phase)
            # vy = (self.arena_scale * 0.6) * self.speed * math.cos(self.speed * t + phase)

            # d.x = 9.94 + ff
            # d.y = 1.30 + ff
            # d.vx = float(vx)
            # d.vy = float(vy)

            # timestamp (float) come nel tuo retriever
            # d.ts = int(t)
            # ff = ff + 0.1
            detections.append(d)

        print("publishing detections:", detections)
        msg.detections = detections
        self.position_count += 1
        if self.position_count >= self.num_points_per_person:
            self.id_counter += 1
        self.position_count = self.position_count % self.num_points_per_person
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeStaticTracksPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
