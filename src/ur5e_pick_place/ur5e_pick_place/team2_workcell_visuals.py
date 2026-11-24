"""RViz-only sheet and camera visuals, read from the Gazebo world.

Does not modify Gazebo, perception, TF, or the MoveIt collision scene.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def apply_pose(marker, text):
    x, y, z, roll, pitch, yaw = map(float, text.split())
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    q = marker.pose.orientation
    q.x, q.y = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy
    q.z, q.w = cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy


class WorkcellVisuals(Node):
    def __init__(self):
        super().__init__('team2_workcell_visuals')
        share = Path(get_package_share_directory('ur5e_pick_place'))
        self.declare_parameter('world_file', str(share/'worlds/workcell.sdf'))
        world = ET.parse(self.get_parameter('world_file').value).getroot().find('world')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/team2/workcell_markers', qos)
        self.markers = MarkerArray()
        camera = world.find("model[@name='top_camera']")
        if camera is not None:
            visual = camera.find('link/visual')
            m = self.marker('camera_body', Marker.CUBE)
            apply_pose(m, camera.findtext('pose', '0 0 0 0 0 0'))
            m.scale.x, m.scale.y, m.scale.z = map(float, visual.findtext('geometry/box/size').split())
            m.color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
            self.markers.markers.append(m)
        mat = world.find("include[name='work_mat']")
        if mat is not None:
            # SDF plane dimensions and the same PNG drive the RViz drawing too.
            # Colored triangles approximate its texture; this is display only.
            model = ET.parse(share/'models/work_mat/model.sdf').getroot()
            width, height = map(float, model.findtext('.//plane/size').split())
            image = cv2.imread(str(share/'models/work_mat/work_sheet.png'))
            if image is None:
                raise RuntimeError('work_sheet.png could not be loaded')
            image = cv2.resize(image, (96, 68), interpolation=cv2.INTER_AREA)
            m = self.marker('work_mat', Marker.TRIANGLE_LIST)
            apply_pose(m, mat.findtext('pose', '0 0 0 0 0 0'))
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.a = 1.0
            for row in range(68):
                for col in range(96):
                    x0, x1 = (col/96-.5)*width, ((col+1)/96-.5)*width
                    y0, y1 = (.5-row/68)*height, (.5-(row+1)/68)*height
                    b, g, r = (float(v)/255 for v in image[row, col])
                    color = ColorRGBA(r=r, g=g, b=b, a=1.0)
                    for x,y in [(x0,y0),(x0,y1),(x1,y1),(x0,y0),(x1,y1),(x1,y0)]:
                        m.points.append(Point(x=x,y=y,z=0.0));m.colors.append(color)
            self.markers.markers.append(m)
        self.publish()
        self.create_timer(5.0, self.publish)
        self.get_logger().info('RViz sheet and camera visuals ready (display only)')

    def marker(self, name, kind):
        m = Marker()
        m.header.frame_id = 'world'
        m.ns, m.id, m.type, m.action = name, 0, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish(self):
        self.publisher.publish(self.markers)


def main():
    rclpy.init()
    node = WorkcellVisuals()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # ROS 신호 처리기가 먼저 context를 종료했어도 중복 종료 오류를 내지 않는다.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
