import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32
import math

class masterodomMecanum(Node):
    def __init__(self):
        super().__init__('Robo_Master_Odom_Mecanum')
        # publish ไปที่ 'odom/unfiltered' -> เป็น input ดิบให้ EKF (robot_localization)
        # เอาไป fuse กับ imu/data แล้ว EKF จะเป็นคนเดียวที่ broadcast TF สุดท้าย
        self.odom_pub = self.create_publisher(Odometry, 'odom/unfiltered', 10)
        # หมายเหตุ: ตัดการ broadcast TF ของ node นี้ออกทั้งหมด
        # (ไม่ใช้ TransformBroadcaster แล้ว) เพราะถ้า node นี้ broadcast TF เอง
        # มุมหันของหุ่นจะมาจาก encoder ล้วนๆ (ไม่เอา IMU มาช่วยแก้เลย)
        # ต้องปล่อยให้ EKF เป็นคน broadcast TF แทน เพราะ EKF จะ fuse
        # ค่า orientation จาก imu/data เข้ามาด้วย

        # Encoder ค่าปัจจุบัน
        self.enc_fl = 0
        self.enc_fr = 0
        self.enc_bl = 0
        self.enc_br = 0

        # Encoder ค่าก่อนหน้า
        self.prev_enc_fl = 0
        self.prev_enc_fr = 0
        self.prev_enc_bl = 0
        self.prev_enc_br = 0

        # ตำแหน่งเริ่มต้น
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.wheel_radius = 0.03  # หน่วย: เมตร
        self.lx = 0.105            # ระยะจาก center ไปล้อหน้า/หลัง
        self.ly = 0.10            # ระยะจาก center ไปล้อซ้าย/ขวา
        self.ticks_per_rev = 1198 # encoder resolution
        self.wheel_circumference = 2 * math.pi * self.wheel_radius

        # ---- Covariance ----
        # odom0_config ที่ใช้ตอนนี้ fuse แค่ vx, vy, vyaw (index 6,7,11 ใน
        # 6x6 twist matrix แบบ row-major: [x,y,z,roll,pitch,yaw])
        # ต้องใส่ค่า variance จริง ไม่ปล่อย 0 เพราะ robot_localization
        # จะตีความ 0 เป็น "มั่นใจสุดขีด/ไม่มี noise" ทำให้ Kalman gain
        # ผิดเพี้ยนหรือ reject การ fuse ไปเลย
        VX_VAR    = 0.001    # m/s, จาก encoder ค่อนข้างแม่น
        VY_VAR    = 0.001
        VYAW_VAR  = 0.01     # rad/s, มี slip มากกว่า vx/vy เลยให้ variance สูงกว่า
        UNUSED    = 1e6      # แกนที่ไม่ได้วัด/ไม่เชื่อถือ (vz, vroll, vpitch)

        self.twist_covariance = [0.0] * 36
        self.twist_covariance[0]  = VX_VAR    # vx
        self.twist_covariance[7]  = VY_VAR    # vy
        self.twist_covariance[14] = UNUSED    # vz
        self.twist_covariance[21] = UNUSED    # vroll
        self.twist_covariance[28] = UNUSED    # vpitch
        self.twist_covariance[35] = VYAW_VAR  # vyaw

        # pose ไม่ได้ถูก fuse (odom0_config ปิดหมดสำหรับ x,y,z,roll,pitch,yaw)
        # แต่ยังใส่ค่าที่สมเหตุสมผลไว้เผื่ออนาคตเปิดใช้/มี node อื่นมาอ่านต่อ
        self.pose_covariance = [0.0] * 36
        self.pose_covariance[0]  = 0.01   # x
        self.pose_covariance[7]  = 0.01   # y
        self.pose_covariance[14] = UNUSED # z
        self.pose_covariance[21] = UNUSED # roll
        self.pose_covariance[28] = UNUSED # pitch
        self.pose_covariance[35] = 0.05   # yaw

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update_odometry)

        self.create_subscription(Int32, 'microRos_ENC_FL', self.fl_callback, 10)
        self.create_subscription(Int32, 'microRos_ENC_FR', self.fr_callback, 10)
        self.create_subscription(Int32, 'microRos_ENC_BL', self.bl_callback, 10)
        self.create_subscription(Int32, 'microRos_ENC_BR', self.br_callback, 10)

    def fl_callback(self, msg): self.enc_fl = msg.data
    def fr_callback(self, msg): self.enc_fr = msg.data
    def bl_callback(self, msg): self.enc_bl = msg.data
    def br_callback(self, msg): self.enc_br = msg.data

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        # delta encoder
        d_fl = self.enc_fl - self.prev_enc_fl
        d_fr = self.enc_fr - self.prev_enc_fr
        d_bl = self.enc_bl - self.prev_enc_bl
        d_br = self.enc_br - self.prev_enc_br

        self.prev_enc_fl = self.enc_fl
        self.prev_enc_fr = self.enc_fr
        self.prev_enc_bl = self.enc_bl
        self.prev_enc_br = self.enc_br

        # แปลงเป็นระยะทาง
        meter_per_tick = self.wheel_circumference / self.ticks_per_rev
        v_fl = d_fl * meter_per_tick / dt
        v_fr = d_fr * meter_per_tick / dt
        v_bl = d_bl * meter_per_tick / dt
        v_br = d_br * meter_per_tick / dt

        # Inverse Kinematics Mecanum
        vx = (v_fl + v_fr + v_bl + v_br) / 4.0
        vy = (-v_fl + v_fr + v_bl - v_br) / 4.0
        vth = (-v_fl + v_fr - v_bl + v_br) / (4.0 * (self.lx + self.ly))

        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        # หมายเหตุ: self.th ตัวนี้คำนวณจาก encoder ล้วนๆ ยังคงเก็บไว้ใช้
        # ในสูตร integrate ตำแหน่ง (x,y) เท่านั้น แต่ orientation ที่ publish
        # ออกไปใน Odometry message ด้านล่างจะให้ EKF เป็นคนตัดสินใจแทน
        # (EKF จะ fuse ค่านี้กับ imu/data เพื่อได้ orientation ที่แม่นกว่า)
        self.th += delta_th

        odom_quat = Quaternion()
        odom_quat.z = math.sin(self.th / 2.0)
        odom_quat.w = math.cos(self.th / 2.0)

        # ---- ส่ง Odometry message เท่านั้น (ไม่ broadcast TF เอง) ----
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.pose.covariance = self.pose_covariance
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = self.twist_covariance
        self.odom_pub.publish(odom)

        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = masterodomMecanum()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()