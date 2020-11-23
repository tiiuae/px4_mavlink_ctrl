import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped



def deg2rad(x):
    return x/360*2*math.pi

def rad2deg(x):
    return x/(2*math.pi)*360

def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll = deg2rad(roll_deg)
    pitch = deg2rad(pitch_deg)
    yaw = deg2rad(yaw_deg)

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


def quaternion_to_euler(x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return rad2deg(X),rad2deg(Y),rad2deg(Z)

class PathPublisherNode(Node):
    def __init__(self):
        self.wp = [
            # lat, lon, rel-alt, yaw at wp, time at wp (not used currently)
            (61.5022353,23.7748721,30.0,90,100e6),
            (61.5027688,23.7750600,30.0,110,200e6),
            (61.5004330,23.7752644,30.0,230,300e6),
            (61.5004168,23.7747874,30.0,90,400e6),
            (61.5025846,23.7736515,30.0,90,500e6),
            (61.5027580,23.7749010,30.0,90,600e6)]

        super().__init__("path_publisher")
        self.publisher = self.create_publisher(
            Path, "path", 10)
        #self.timer = self.create_timer(
        #    2.0, self.publish(1)) # todo, publish one-by-one

    def publish(self, no = None):
        path = Path()
        path.header.stamp = rclpy.clock.Clock().now().to_msg()
        path.header.frame_id = "map"

        if no:
            print ("Publish single wp")
            # publish 1 wp
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp # add time offset
            pose.header.frame_id = "map"
            point = Point()
            point.x = self.wp[no][0]
            point.y = self.wp[no][1]
            point.z = self.wp[no][2]
            pose.pose.position = point

            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = euler_to_quaternion(0,0,self.wp[no][3])
            path.poses.append(pose)

            #print('Publishing: "%s"' % path.poses)
            self.publisher.publish(path)
        else:
            print ("Publish path, no of waypoints:",len(self.wp))
            # publish all wps
            for i in range(0,len(self.wp)):
                pose = PoseStamped()
                pose.header.stamp = path.header.stamp # add time offset
                pose.header.frame_id = "map"
                point = Point()
                point.x = self.wp[i][0]
                point.y = self.wp[i][1]
                point.z = self.wp[i][2]
                pose.pose.position = point

                pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = euler_to_quaternion(0,0,self.wp[i][3])
                path.poses.append(pose)
            print('Publishing: "%s"' % path.poses)
            self.publisher.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()
    #rclpy.spin(node)
    node.publish()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

