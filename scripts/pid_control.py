#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class PIDGyroToVel:
    def __init__(self):
        rospy.init_node("pid_gyro_to_vel", anonymous=True)
        self.pub_cmd = rospy.Publisher('/vel/cmd', Twist, queue_size=10)
        self.sub_gyro = rospy.Subscriber('/gyro/data', Float32, self.gyro_cb)

        # PID Parameter
        self.kp = 1.0
        self.ki = 0.001
        self.kd = 0.9
        self.prev_error = 0.0
        self.integral = 0.0
        self.current_pitch = 0.0
        self.last_time = rospy.get_time()
        rospy.loginfo("PID Controller Aktif")
        self.loop()

    def gyro_cb(self, msg):
        self.current_pitch = msg.data

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            twist = Twist()
            dt = now - self.last_time
            if dt == 0:
                dt = 0.01
            error = 0.0 - self.current_pitch
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.prev_error = error
            self.last_time = now
            base_speed = 60.0 
            speed_kiri = base_speed - output
            speed_kanan = base_speed + output
            speed_kiri = max(20.0, min(100.0, speed_kiri))
            speed_kanan = max(20.0, min(100.0, speed_kanan))
            twist.linear.x = speed_kiri
            twist.angular.x = speed_kanan
            self.pub_cmd.publish(twist)
            print(f"[IMU] :\n Pitch: {self.current_pitch:.2f}\n Error: {error:.2f}\n")
            print(f"[PID] :\n Output: {output:.2f}\n Kiri: {speed_kiri:.1f}\n Kanan: {speed_kanan:.1f}\n")
            rate.sleep()

if __name__ == '__main__':
    try:
        PIDGyroToVel()
    except rospy.ROSInterruptException:
        pass
