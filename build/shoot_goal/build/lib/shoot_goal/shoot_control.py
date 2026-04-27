import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from protocol.msg import MotionServoCmd
import math

def changeJingDu(x):
    if x>0:
        return -math.pi+x
    else:
        return math.pi+x

class ShootController(Node):
    def __init__(self):
        super().__init__('shoot_controller')

        # =========================
        # ROS 参数（动捕刚体名 tongtong=狗，ball=球；运动话题与狗命名空间 dog1 对齐）
        # 启动示例：
        #   ros2 run ... --ros-args -p dog_pose_topic:=/vrpn/tongtong/pose ...
        # =========================
        self.declare_parameter('dog_pose_topic', '/vrpn/tongtong/pose')
        self.declare_parameter('ball_pose_topic', '/vrpn/ball/pose')
        self.declare_parameter('motion_servo_cmd_topic', '/dog1/motion_servo_cmd')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 5.0)

        dog_pose_topic = self.get_parameter('dog_pose_topic').get_parameter_value().string_value
        ball_pose_topic = self.get_parameter('ball_pose_topic').get_parameter_value().string_value
        motion_cmd_topic = self.get_parameter('motion_servo_cmd_topic').get_parameter_value().string_value
        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value

        # =========================
        # 场地参数
        # =========================
        self.goal_position = (goal_x, goal_y)

        # 球门宽度的一半
        self.goal_half_width = 0.8

        # 机器狗目标点距离球的后方距离
        # 如果机器狗定位点在身体中心，0.3 可能略近，可以根据实际改为 0.35~0.45
        self.desired_distance = 0.5

        # =========================
        # 控制参数
        # =========================
        self.linear_speed = 0.45            # 最大前进速度 m/s，推球建议不要太快
        self.angular_gain = 0.7              # 角速度比例系数
        self.linear_gain = 0.6               # 距离比例系数

        self.position_tolerance = 0.15       # 到达球后方目标点的位置误差
        self.angle_tolerance = 0.15          # 对准球门角度阈值，单位 rad

        self.control_rate = 20.0             # 控制频率 20Hz

        # 角速度限幅
        self.max_wz_goto = 0.7
        self.max_wz_align = 0.6

        # 当角度误差较大时，先原地转向，不前进
        self.turn_in_place_threshold = 0.5   # rad，大约 23 度

        # 推球过程中，如果狗-球方向和球-门方向偏差过大，则重新绕到球后
        self.push_alignment_tolerance = 0.35  # rad，大约 20 度

        # 如果推球阶段狗距离球太远，说明没推到球或球跑远了，重新定位
        self.max_push_distance = 1.0

        # =========================
        # 状态机
        # GOTO_BEHIND_BALL -> ALIGN_TO_GOAL -> PUSH_TO_GOAL -> DONE
        # =========================
        self.state = 'GOTO_BEHIND_BALL'

        self.start_time = None
        self.log_count = 0

        # =========================
        # 订阅 VRPN 位姿
        # =========================
        self.dog_sub = self.create_subscription(
            PoseStamped,
            dog_pose_topic,
            self.dog_pose_callback,
            10
        )

        self.ball_sub = self.create_subscription(
            PoseStamped,
            ball_pose_topic,
            self.ball_pose_callback,
            10
        )

        # =========================
        # 发布运动指令
        # =========================
        self.cmd_pub = self.create_publisher(
            MotionServoCmd,
            motion_cmd_topic,
            10
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        self.dog_pose = None
        self.ball_pose = None

        self.wz_sign=1.0

        self.get_logger().info(
            f'Shoot controller started. dog={dog_pose_topic}, ball={ball_pose_topic}, '
            f'cmd={motion_cmd_topic}, goal=({goal_x}, {goal_y}). State: GOTO_BEHIND_BALL'
        )

    # =========================
    # 回调函数
    # =========================
    def dog_pose_callback(self, msg):

        self.dog_pose = msg.pose

    def ball_pose_callback(self, msg):
        self.ball_pose = msg.pose.position

    # =========================
    # 工具函数
    # =========================
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def log_throttle(self, msg, period_count=20, level='info'):
        """
        控制日志频率。
        period_count=20 时，在 20Hz 控制频率下大约每秒打印一次。
        """
        if self.log_count % period_count == 0:
            if level == 'warn':
                self.get_logger().warn(msg)
            elif level == 'error':
                self.get_logger().error(msg)
            else:
                self.get_logger().info(msg)

    # =========================
    # 主控制循环
    # =========================
    def control_loop(self):
        self.log_count += 1

        # 等待动捕数据
        if self.dog_pose is None or self.ball_pose is None:
            self.log_throttle('Waiting for dog pose and ball pose...', period_count=40)
            return

        # 收到第一帧有效数据后再开始计时
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info('Received pose data. Start 60s timer.')

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        # 1 分钟超时
        if elapsed > 60.0:
            self.get_logger().warn('Time out! Stop.')
            self.stop_robot()
            self.timer.cancel()
            return

        # 当前机器狗位置
        dog_x = self.dog_pose.position.x
        dog_y = self.dog_pose.position.y
        dog_yaw = self.quaternion_to_yaw(self.dog_pose.orientation)

        # 当前球位置
        ball_x = self.ball_pose.x
        ball_y = self.ball_pose.y

        # 球门位置
        goal_x, goal_y = self.goal_position

        # =====================================================
        # 进球检测
        # 保持你的原始逻辑：
        # 假设 ball_y >= goal_y 表示球越过球门线；
        # abs(ball_x - goal_x) <= goal_half_width 表示球在门柱范围内。

        # =====================================================
        if ball_y >= goal_y and abs(ball_x - goal_x) <= self.goal_half_width:
            self.get_logger().info('GOAL! Task succeeded.')
            self.stop_robot()
            self.state = 'DONE'
            self.timer.cancel()
            return

        # =====================================================
        # 计算球 -> 球门方向
        # =====================================================
        goal_dx = goal_x - ball_x
        goal_dy = goal_y - ball_y
        goal_dist = math.hypot(goal_dx, goal_dy)

        if goal_dist < 1e-6:
            self.get_logger().info('Ball is already near goal center. Stop.')
            self.stop_robot()
            return

        unit_goal_x = goal_dx / goal_dist
        unit_goal_y = goal_dy / goal_dist

        # =====================================================
        # 状态 1：移动到球后方
        # =====================================================
        if self.state == 'GOTO_BEHIND_BALL':
            # 球后方目标点：沿着球门反方向退 desired_distance
            desired_x = ball_x - unit_goal_x * self.desired_distance+0.05
            desired_y = ball_y - unit_goal_y * self.desired_distance

            dx = desired_x - dog_x
            dy = desired_y - dog_y
            dist = math.hypot(dx, dy)

            target_angle = changeJingDu(math.atan2(dy, dx))
            angle_error = self.normalize_angle(target_angle - dog_yaw)

            self.log_throttle(
                f'[GOTO_BEHIND_BALL] dist={dist:.2f}, angle_err={angle_error:.2f}',
                period_count=20
            )

            # 到达球后方目标点
            if dist < self.position_tolerance:
                self.stop_robot()
                self.state = 'ALIGN_TO_GOAL'
                self.get_logger().info('Reached behind ball. Switch to ALIGN_TO_GOAL.')
                return

            # 角速度控制
            wz = self.angular_gain * angle_error
            wz = self.clamp(wz, -self.max_wz_goto, self.max_wz_goto)

            # 如果朝向误差较大，先原地转向
            if abs(angle_error) > self.turn_in_place_threshold:
                vx = 0.0
            else:
                vx = self.linear_gain * dist
                vx = self.clamp(vx, 0.0, self.linear_speed)

            self.move(vx, wz)

        # =====================================================
        # 状态 2：在球后方对准球门
        # =====================================================
        elif self.state == 'ALIGN_TO_GOAL':
            desired_yaw = changeJingDu(math.atan2(goal_dy, goal_dx))
            angle_error = self.normalize_angle(desired_yaw - dog_yaw)

            self.log_throttle(
                f'[ALIGN_TO_GOAL] angle_err={angle_error:.2f}',
                period_count=20
            )

            if abs(angle_error) > self.angle_tolerance:
                wz = self.angular_gain * angle_error
                wz = self.clamp(wz, -self.max_wz_align, self.max_wz_align)
                self.move(0.0, wz)
            else:
                self.stop_robot()
                self.state = 'PUSH_TO_GOAL'
                self.get_logger().info('Aligned to goal. Switch to PUSH_TO_GOAL.')

        # =====================================================
        # 状态 3：推球进门
        # =====================================================
        elif self.state == 'PUSH_TO_GOAL':
            # 狗到球的向量
            dog_to_ball_x = ball_x - dog_x
            dog_to_ball_y = ball_y - dog_y
            dog_to_ball_dist = math.hypot(dog_to_ball_x, dog_to_ball_y)

            if dog_to_ball_dist < 1e-6:
                dog_to_ball_angle = dog_yaw
            else:
                dog_to_ball_angle = math.atan2(dog_to_ball_y, dog_to_ball_x)

            # 球到球门方向
            goal_angle = changeJingDu(math.atan2(goal_dy, goal_dx))

            # 判断狗-球方向是否仍然和球-门方向大致一致
            push_alignment_error = changeJingDu(self.normalize_angle(dog_to_ball_angle - goal_angle))

            # 同时也检查机器狗自身朝向是否对准球门
            yaw_error = self.normalize_angle(goal_angle - dog_yaw)

            self.log_throttle(
                f'[PUSH_TO_GOAL] dog_ball_dist={dog_to_ball_dist:.2f}, '
                f'push_align_err={push_alignment_error:.2f}, yaw_err={yaw_error:.2f}',
                period_count=20
            )

            # 如果狗已经偏离球后方，重新绕到球后
            if abs(push_alignment_error) > self.push_alignment_tolerance:
                self.get_logger().warn('Lost ball-goal alignment. Reposition behind ball.')
                self.stop_robot()
                self.state = 'GOTO_BEHIND_BALL'
                return

            # 如果距离球太远，说明没有稳定推到球，重新定位
            if dog_to_ball_dist > self.max_push_distance:
                self.get_logger().warn('Dog is too far from ball. Reposition behind ball.')
                self.stop_robot()
                self.state = 'GOTO_BEHIND_BALL'
                return

            # 如果推球过程中机身朝向偏了，先小幅修正
            if abs(yaw_error) > self.angle_tolerance:
                wz = self.angular_gain * yaw_error
                wz = self.clamp(wz, -self.max_wz_align, self.max_wz_align)
                self.move(0.0, wz)
                return

            # 正式推球
            self.move(self.linear_speed, 0.0)

        # =====================================================
        # 状态 4：完成
        # =====================================================
        elif self.state == 'DONE':
            self.stop_robot()

        else:
            self.get_logger().warn(f'Unknown state: {self.state}. Stop robot.')
            self.stop_robot()
            self.state = 'DONE'

    # =========================
    # 运动控制
    # =========================
    def move(self, vx, wz):

        cmd = MotionServoCmd()
        cmd.motion_id = 303
        cmd.cmd_type = 1
        cmd.value = 2

        # vel_des = [前进速度, 横向速度, 角速度]
        cmd.vel_des = [float(vx), 0.0, float(self.wz_sign * wz)]

        cmd.step_height = [0.05, 0.05]
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = MotionServoCmd()
        cmd.motion_id = 303
        cmd.cmd_type = 1
        cmd.value = 2
        cmd.vel_des = [0.0, 0.0, 0.0]
        cmd.step_height = [0.05, 0.05]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = ShootController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Stop robot.')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()