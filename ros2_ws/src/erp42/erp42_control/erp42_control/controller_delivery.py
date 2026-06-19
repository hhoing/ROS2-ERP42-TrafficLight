import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import numpy as np
from DB import DB
from rclpy.qos import qos_profile_system_default
from erp42_msgs.msg import ControlMessage
from stanley import Stanley
import time


"""
10.25 테스트용 사용법
1. gps 잘 터지는 곳으로 간다 (매우 중요)
2. global path 하나, devliery path 3개, delivery_back_path 3개 총 7개의 db를 녹화한다.
3. state machine을 driving_a -> delivery_a -> driving_b 로 바꾼다.
4. 코드들 다 키고 알고리즘 정상 작동 여부 확인한다. 
(켜야하는 코드 중에 모르는 거 있으면 연락주세요)

** 알아두면 좋을 것: gear=0 전진 gear=2 후진 estop=1 활성화 estop=0 비활성화
** Gain 값 조절 방법: stanley_control() 함수에 h_gain, c_gain 값이 있는데, 초기값을 기록해 놓고 필요에 따라 계속 수정해줘야함.
                    예를들어 현재 delivery path 의 경우 생성된 경로를 odometry 가 잘 추종하지 못하는 현상이 있기 때문에
                    두 게인값을 모두 지금보다 조금 올려서 재실험 해봐야함. speed는 3km/h 정도로 충분히 낮게.
                    driving 도 학교 실험 환경에서는 그냥 4~5km/h 정도로 ㄱㄱ
"""
# gear=2: 전진 , gear=0: 후진 (수정1025)

class Delivery:
    def __init__(self, node):
        self.node = node
        self.globalpath_sub = self.node.create_subscription(
            Path,
            "global_path",
            self.callback_global,
            qos_profile=qos_profile_system_default,
        )

        self.delivery_path_pub = self.node.create_publisher(Path, "/delivery_path", 10)

        # Stanley 객체 초기화
        self.st = Stanley()  # Stanley 컨트롤러 인스턴스 생성

        # Lists for storing global path data
        self.gx_list = []
        self.gy_list = []
        self.gyaw_list = []

        # DB 속성들을 먼저 초기화
        self.abs1_path_db = DB("A1.db")
        # self.abs2_path_db = DB("B2_accurate_acca.db")
        # self.abs3_path_db = DB("B3_accurate_acca.db")

        # 추가: abs_back 경로를 위한 DB 초기화
        self.abs1_back_path_db = DB("A1_bk.db")
        # self.abs2_back_path_db = DB("B2_back_accurate_acca.db")
        # self.abs3_back_path_db = DB("B3_back_accurate_acca.db")

        # 데이터베이스에서 모든 경로를 한 번에 읽어옴
        self.abs1_path = np.array(self.abs1_path_db.read_db_n("Path", "x", "y", "yaw"))
        # self.abs2_path = np.array(self.abs2_path_db.read_db_n("Path", "x", "y", "yaw"))
        # self.abs3_path = np.array(self.abs3_path_db.read_db_n("Path", "x", "y", "yaw"))

        # 추가: abs_back 경로 로드
        self.abs1_back_path = np.array(
            self.abs1_back_path_db.read_db_n("Path", "x", "y", "yaw")
        )
        # self.abs2_back_path = np.array(
        #     self.abs2_back_path_db.read_db_n("Path", "x", "y", "yaw")
        # )
        # self.abs3_back_path = np.array(
        #     self.abs3_back_path_db.read_db_n("Path", "x", "y", "yaw")
        # )

        # 경로 선택 (abs1, abs2, abs3 중 하나 선택)
        self.abs_var = "abs1"  # "abs1" or "abs2" or "abs3"
        self.db = self.select_path(self.abs_var)

        # 추가: 후진 경로 선택
        self.back_db = self.select_back_path(self.abs_var)

        self.estop = 0
        self.target_idx = 0
        self.delivery_finished = False
        self.path_published_once = False  # 경로가 한 번만 publish 되도록 설정

        # 추가: 후진 경로 사용 여부 및 기어 상태
        self.is_back_path = False
        # self.gear = 0  # 전진 기어 (0: 전진, 1: 중립, 2: 후진)
        self.gear = 2
        self.count = 0  # estop 유지 카운터 초기화

    def select_path(self, abs_var):
        """abs_var에 따라 선택된 경로를 반환"""
        if abs_var == "abs1":
            return self.abs1_path
        elif abs_var == "abs2":
            return self.abs2_path
        elif abs_var == "abs3":
            return self.abs3_path
        else:
            raise ValueError(f"Invalid abs_var: {abs_var}")

    def select_back_path(self, abs_var):
        """abs_var에 따라 선택된 후진 경로를 반환"""
        if abs_var == "abs1":
            return self.abs1_back_path
        elif abs_var == "abs2":
            return self.abs2_back_path
        elif abs_var == "abs3":
            return self.abs3_back_path
        else:
            raise ValueError(f"Invalid abs_var: {abs_var}")

    def callback_global(self, msg):
        for p in msg.poses:
            self.gx_list.append(p.pose.position.x)
            self.gy_list.append(p.pose.position.y)
            _, _, yaw = euler_from_quaternion(
                [
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                ]
            )
            self.gyaw_list.append(yaw)

    def control_delivery(self, odometry):
        msg = ControlMessage()

        if not self.path_published_once:
            self.publish_path()

        if len(self.db) != 0:  # 경로가 있으면
            # x, y, yaw 데이터를 db에서 가져옴
            path_cx = self.db[:, 0]
            path_cy = self.db[:, 1]
            path_cyaw = self.db[:, 2]

            # Stanley control을 사용하여 제어 계산
            steer, self.target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                path_cx,
                path_cy,
                path_cyaw,
                h_gain=0.5,
                c_gain=0.24,
            )

            print("target_idx of delivery_path: ", self.target_idx)

            # 경로의 마지막 idx에 도달했는지 확인
            if self.target_idx >= len(path_cx) - 2:
                print(f"Target index reached: {self.target_idx}/{len(path_cx)}")
                if self.count <= 50:  # estop을 유지하는 카운트 (예: 50회)
                    self.estop = 1
                    self.count += 1
                    self.node.get_logger().info(f"Stopping... count: {self.count}")
                else:
                    self.estop = 0
                    if not self.is_back_path:
                        # 후진 경로로 전환
                        self.db = self.back_db
                        self.target_idx = 0  # 인덱스 초기화
                        self.is_back_path = True
                        # self.gear = 2  # 후진 기어로 설정 (2: 후진)
                        self.gear = 0
                        self.path_published_once = (
                            False  # 경로를 다시 publish하기 위해 플래그 초기화
                        )
                        self.publish_path()  # 후진 경로 publish
            self.speed = 3  # 기본 속도 유지

        else:
            print("No delivery path")
            self.estop = 0

        # Control 메시지 생성
        msg.steer = int(math.degrees((-1) * steer))
        msg.speed = self.speed * 10
        msg.gear = self.gear  # 기어 설정 (전진 또는 후진)
        msg.estop = self.estop

        if self.is_back_path and self.target_idx >= len(self.db[:, 0]) - 5:
            self.delivery_finished = True  # 후진 경로의 마지막에 도달하면 종료
            self.node.get_logger().info("Mission Finished.")

        print("Delivery has finished")

        return msg, self.delivery_finished

    def publish_path(self):
        path_data = self.db

        if path_data is not None and len(path_data) > 0:
            path_msg = Path()
            path_msg.header.stamp = self.node.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"

            for x, y, yaw in path_data:
                pose = PoseStamped()
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quaternion = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path_msg.poses.append(pose)

            self.delivery_path_pub.publish(path_msg)
            self.path_published_once = True
            print("Path published successfully.")
        else:
            self.node.get_logger().error("Failed to load path from database")
            print("Error: No path found in the database.")
