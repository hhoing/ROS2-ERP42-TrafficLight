import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

class ImageConvertor(Node):
    def __init__(self):
        super().__init__('ImageConverter')  # 노드 이름 설정
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, 'lane', 10)
        self.coord_publisher = self.create_publisher(Float32MultiArray, 'laneCoordinate', 10)
        self.subscription
        self.bridge = CvBridge()
        self.previous_left_lane = None  # 이전 왼쪽 차선의 기울기와 절편
        self.previous_right_lane = None  # 이전 오른쪽 차선의 기울기와 절편
        self.alpha = 0.6  # 이전 프레임 정보를 활용해 순간 검출 실패나 흔들림을 완화
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # 이미지 처리 및 차선 검출 수행
        processed_image = self.process_image(cv_image)
        # 검출된 차선이 표시된 이미지를 발행
        self.publish_image(processed_image)
        
    def process_image(self, image):
        gray_img = self.grayscale(image)
        smoothed_img = self.gaussian_blur(img=gray_img, kernel_size=5)
        canny_img = self.canny(img=smoothed_img, low_threshold=180, high_threshold=240)
        imshape = image.shape
        vertices = np.array([[(0, imshape[0] * 0.74), (imshape[1] * 0.39, imshape[0] * 0.47),
                              (imshape[1] * 0.61, imshape[0] * 0.47), (imshape[1], imshape[0] * 0.74)]],
                            dtype=np.int32)
        masked_img = self.region_of_interest(img=canny_img, vertices=vertices)
        rho = 1
        theta = np.pi / 180
        threshold = 20
        min_line_len = 20
        max_line_gap = 180
        lines, line_img = self.hough_lines(img=masked_img, rho=rho, theta=theta, threshold=threshold,
                                           min_line_len=min_line_len, max_line_gap=max_line_gap)
        output = self.slope_lines(image=image, lines=lines)
        return output
    
    def grayscale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def canny(self, img, low_threshold, high_threshold):
        return cv2.Canny(img, low_threshold, high_threshold)

    def gaussian_blur(self, img, kernel_size):
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def region_of_interest(self, img, vertices):
        mask = np.zeros_like(img)
        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap):
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]),
                                min_line_len, max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        return lines, line_img

    def slope_lines(self, image, lines):
        if lines is None:
            self.publish_lane_coordinates(None, None)
            return image

        img = np.copy(image)  # 원본 이미지 보존
        left_lane_lines = []  # 왼쪽 차선 후보의 기울기와 절편
        right_lane_lines = []  # 오른쪽 차선 후보의 기울기와 절편
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 == x2: continue  # 수직 선 무시
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope * x1
                if abs(slope) > 0.5:  # 기울기 필터링
                    if slope < 0:  # 좌우 차선을 기울기 기준으로 분리
                        left_lane_lines.append((slope, intercept))
                    else:
                        right_lane_lines.append((slope, intercept))

        # 차선 평균값 계산
        left_lane = np.mean(left_lane_lines, axis=0) if left_lane_lines else None
        right_lane = np.mean(right_lane_lines, axis=0) if right_lane_lines else None

        # 이전 프레임의 차선 정보와 현재 검출 결과를 결합
        if left_lane is not None and self.previous_left_lane is not None:
            left_lane = left_lane * (1 - self.alpha) + self.previous_left_lane * self.alpha
        elif self.previous_left_lane is not None:
            left_lane = self.previous_left_lane

        if right_lane is not None and self.previous_right_lane is not None:
            right_lane = right_lane * (1 - self.alpha) + self.previous_right_lane * self.alpha
        elif self.previous_right_lane is not None:
            right_lane = self.previous_right_lane

        # 평균 차선을 그리고, 실제로 그린 좌표를 반환받음
        left_coords = self.draw_lane(img, left_lane)
        right_coords = self.draw_lane(img, right_lane)

        # 검출된 평균 차선 좌표를 laneCoordinate 토픽으로 발행
        self.publish_lane_coordinates(left_coords, right_coords)

        # 현재 차선 정보 저장
        self.previous_left_lane = left_lane if left_lane is not None else self.previous_left_lane
        self.previous_right_lane = right_lane if right_lane is not None else self.previous_right_lane

        return cv2.addWeighted(image, 0.7, img, 0.3, 0)


    def draw_lane(self, img, lane, extrapolate=False):
        if lane is None:
            return None

        slope, intercept = lane
        if abs(slope) < 1e-6:
            return None

        y1 = img.shape[0]  # 이미지의 맨 아래
        y2 = int(y1 * 0.45)  # ROI 상단 근처까지 차선을 표시

        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        x1 = int(np.clip(x1, 0, img.shape[1] - 1))
        x2 = int(np.clip(x2, 0, img.shape[1] - 1))

        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 10)
        return (x1, y1, x2, y2)

    def publish_lane_coordinates(self, left_coords, right_coords):
        def coords_or_default(coords):
            if coords is None:
                return [-1.0, -1.0, -1.0, -1.0]
            return [float(value) for value in coords]

        coord_msg = Float32MultiArray()
        coord_msg.data = coords_or_default(left_coords) + coords_or_default(right_coords)
        self.coord_publisher.publish(coord_msg)


    def publish_image(self, image):
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.publisher.publish(img_msg)        

        
def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = ImageConvertor()
    rclpy.spin(lane_detection_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
