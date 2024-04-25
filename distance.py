#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import math
from std_msgs.msg import Float32, String
from move_base_msgs.msg import MoveBaseActionGoal
import time

class DistanceCalculator:
    def __init__(self):
        rospy.init_node('distance_calculator', anonymous=True)
        self.current_pose = PoseStamped()
        self.path = Path()
        self.initial_total_distance = None
        self.goal_reached = False
        self.current_speed = 0.0
        self.cancelled_goal = False
        self.last_goal_id = None
        self.plan_subscriber = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.plan_callback)
        self.pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.status_subscriber = rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal , self.goal_callback)
        self.initial_distance_set = False
        self.percentage_publisher = rospy.Publisher('/path_completion_percentage', Float32, queue_size=10)
        self.time_publisher = rospy.Publisher('/estimated_time_remaining', String, queue_size=10)
        self.last_goal_x = None  # Yeni eklenen satır
        self.new_goal_x = None
        self.goal_changed = None

    def pose_callback(self, msg):
        self.current_pose.pose = msg.pose.pose
        if self.path.poses:
            self.calculate_remaining_distance()

    def plan_callback(self, msg):
        print("Plan çağırdım giremedim hedef tamam parametre", self.goal_reached)
        print("Plan çağırdım giremedim hedef iptal parametre", self.cancelled_goal)
        if not self.goal_reached and not self.cancelled_goal:
            print("Yeniden Yol Planlandı.")
            print("Toplam mesafe hesaplanan",self.calculate_total_distance(msg.poses))
            print("İnital Pose Önce",self.initial_total_distance)
            self.initial_total_distance = self.calculate_total_distance(msg.poses)
            self.goal_reached = False
            self.cancelled_goal = False  # İlk hedef atanır atanmaz bu durumu sıfırla
            print("İnital Pose Sonra",self.initial_total_distance)
        self.path = msg

    def status_callback(self, msg):
        if msg.status_list:
            # İlk hedef atanmadıysa ve hedefe ulaşılmadıysa
            if not self.last_goal_id and msg.status_list[0].status == 1:
                self.last_goal_id = msg.status_list[0].goal_id.id
                self.cancelled_goal = False
                self.goal_reached = False
                rospy.loginfo(f"Yeni hedef atanmıştır. Hedef ID: {self.last_goal_id}")
            # Hedef tamamlandığında
            elif self.last_goal_id and msg.status_list[0].status == 3:
                rospy.loginfo("Hedef tamamlandı.")
                self.cancelled_goal = False
                self.goal_reached = False
                self.percentage_publisher.publish(Float32(100))
            # Hedef iptal edildiğinde
            elif self.last_goal_id and msg.status_list[0].status == 2:
                rospy.loginfo("Hedef iptal edildi.")
                if self.goal_changed:
                    self.cancelled_goal = False
                    self.goal_reached = False
                    print("Goal Changed",self.goal_reached)
                else:
                    self.cancelled_goal = True
                    self.goal_reached = False
                    print("Not Goal Changed",self.goal_reached)

                
    def goal_callback(self, msg):
        # Yeni bir hedef alındığında kontrolü yap
        if self.cancelled_goal:
            self.new_goal_x = msg.goal.target_pose.pose.position.x
            print("new_goal_x",self.new_goal_x)
            print("last_goal_x", self.last_goal_x)
            if self.new_goal_x != self.last_goal_x:
                rospy.loginfo(f"Yeni bir hedef atanmıştır. Önceki Hedef X: {self.last_goal_x}")
                self.last_goal_x = self.new_goal_x
                self.goal_reached = False
                self.cancelled_goal = False  # İlk hedef atanır atanmaz bu durumu sıfırla
                self.goal_changed = True
                # self.initial_total_distance = self.calculate_total_distance(self.path.poses)
                rospy.loginfo("Total mesafe sıfırlandı.")
        else: 
            self.last_goal_x = msg.goal.target_pose.pose.position.x
            rospy.loginfo(f"Hedef X: {self.last_goal_x}")
            rospy.loginfo(f"Hedef Xnew: {self.new_goal_x}")
            self.goal_changed = False


    def odom_callback(self, msg):
        # Robotun lineer hızını güncelle
        self.current_speed = msg.twist.twist.linear.x

    def calculate_remaining_distance(self):
        closest_point_index = self.find_closest_path_point_index(self.current_pose.pose, self.path.poses)
        if closest_point_index is not None and self.initial_total_distance is not None:
            remaining_distance = self.calculate_path_distance_from_index(closest_point_index)
            # rospy.loginfo(f"Hedefe kalan mesafe: {remaining_distance:.2f} m")
            # print(remaining_distance)
            # print(self.initial_total_distance)
            percentage_completed = (1 - remaining_distance / self.initial_total_distance) * 100
            self.percentage_publisher.publish(Float32(percentage_completed))
            # print("PERCENTAGE: ",Float32(percentage_completed))
            # Süre hesaplaması ve yayını
            if self.current_speed > 0.01:
                estimated_time_remaining_seconds = remaining_distance / self.current_speed
                minutes = int(estimated_time_remaining_seconds // 60)
                seconds = int(estimated_time_remaining_seconds % 60)
                time_str = f"{minutes} dakika {seconds} saniye"
                self.time_publisher.publish(String(time_str))
                # rospy.loginfo(f"Tahmini kalan süre: {time_str}")

    def find_closest_path_point_index(self, current_pose, path_poses):
        min_distance = float('inf')
        closest_index = None
        for i, pose in enumerate(path_poses):
            distance = self.distance_between_points(current_pose.position, pose.pose.position)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        return closest_index

    def calculate_path_distance_from_index(self, index):
        distance = 0.0
        for i in range(index, len(self.path.poses) - 1):
            distance += self.distance_between_points(self.path.poses[i].pose.position, self.path.poses[i+1].pose.position)
        return distance

    def calculate_total_distance(self, poses):
        total_distance = 0.0
        for i in range(len(poses)-1):
            total_distance += self.distance_between_points(poses[i].pose.position, poses[i+1].pose.position)
        return total_distance

    def distance_between_points(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return math.sqrt(dx**2 + dy**2)

if __name__ == '__main__':
    try:
        distance_calculator = DistanceCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
