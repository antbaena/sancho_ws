import numpy as np
import cv2
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import OperatingModes

from human_face_recognition_msgs.srv import Detection, AudioLocation, Tracking
from human_face_recognition_msgs.msg import FacePosition
from .camera_parameters import RESOLUTION, get_parameters
from .hri_bridge import HRIBridge


class HRIHeadNode(Node):

    def __init__(self, hri_head):
        super().__init__("hri_head")

        self.br = HRIBridge()
        self.hri_head = hri_head

        self.serv_neck_op = self.create_client(
            OperatingModes, "/wxxms/set_operating_modes"
        )
        while not self.serv_neck_op.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("HeadService not available, waiting...")

        self.detection_client = self.create_client(Detection, "detection")
        while not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Detection service not available, waiting again...")

        self.audio_location_client = self.create_client(AudioLocation, "audio_location")
        # while not self.audio_location_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('AudioLocation service not available, waiting again...')

        self.face_tracking_client = self.create_service(
            Tracking, "track", self.tracking_callback
        )
        self.publisher_tracking = self.create_subscription(
            FacePosition, "tracking", self.track_face_callback, 10
        )

        self.pub_neck_joint = self.create_publisher(
            JointGroupCommand, "/wxxms/commands/joint_group", 1
        )
        self.subscription_camera = self.create_subscription(
            Image, "camera/color/image_raw", self.frame_callback, 1
        )
        self.sub_detection_data = self.create_subscription(
            JointState, "/wxxms/joint_states", self.head_position_callback, 10
        )

    def tracking_callback(self, request, response):
        if not self.hri_head.tracking_mode:
            self.hri_head.tracking_mode = True
            response.text = String(data="ON")
        else:
            self.hri_head.tracking_mode = False
            response.text = String(data="OFF")
        return response

    def track_face_callback(self, position_msg):
        if self.hri_head.tracking_mode:
            positions = []
            x = position_msg.x
            y = position_msg.y
            w = position_msg.w
            h = position_msg.h
            positions.append([x, y, w, h])
            self.hri_head.tracked_face = positions

    def frame_callback(self, frame_msg):
        """Callback for camera frames"""

        self.hri_head.last_frame = frame_msg

    def head_position_callback(self, msg):
        self.hri_head.pan = msg.position[0]
        self.hri_head.tilt = msg.position[1]


class HRIHead:

    def __init__(self):
        self.last_frame = None
        self.pan = 0
        self.tilt = 0
        self.tracking_mode = False
        self.tracked_face = None

        self.last_head_move = time.time()
        self.last_face_detection_time = time.time()
        self.last_sound_angle_time = time.time()

        self.dark_saturated_threshold = 50
        self.audio_intensity_threshold = 750
        self.max_delay_seconds_face = 5
        self.max_delay_seconds_sound = 5

        self.camera_matrix, self.camera_matrix_inv, self.dist_coeffs = get_parameters(
            RESOLUTION
        )

        self.node = HRIHeadNode(self)
        self.init_head()

    def spin(self):
        while rclpy.ok():
            if not self.tracking_mode:
                frame_msg = self.last_frame
                frame = (
                    self.node.br.imgmsg_to_cv2(frame_msg, "bgr8")
                    if frame_msg is not None
                    else None
                )

                if frame is not None:
                    image_brightness = self.brightness(frame)

                    if image_brightness < self.dark_saturated_threshold:
                        self.node.get_logger().info(
                            "La imagen es muy oscura, no veo nada "
                            + "(Brillo de la imagen: "
                            + str(int(image_brightness))
                            + " < "
                            + str(self.dark_saturated_threshold)
                            + ")"
                        )

                    positions_msg, scores_msg = self.detection_request(frame_msg)
                    positions, _ = self.node.br.msg_to_detector(
                        positions_msg, scores_msg
                    )
                    self.move_with_face(positions)

                if (
                    time.time() - self.last_face_detection_time
                    > self.max_delay_seconds_face
                ):  # NO Hay caras por más de 5 segundos
                    audio_intensity, sound_angle = self.audio_location_request()
                    self.node.get_logger().info(str(audio_intensity) + " " + str(sound_angle))
                    if audio_intensity != -1:
                        self.move_with_sound(audio_intensity, sound_angle)
            elif self.tracked_face is not None:
                self.move_with_face(self.tracked_face)
                self.tracked_face = None

            rclpy.spin_once(self.node)

    def move_with_face(self, positions):
        if len(positions) > 0:  # SI hay caras en la imagen
            (x, y, w, h) = positions[0]
            head_position = [x + (w / 2), y + (h / 2)]

            # camera_position = [int(frame.shape[1]/2), int(frame.shape[0]/2)]

            head_position_homogeneous = np.array(
                [head_position[0], head_position[1], 1]
            )
            head_position_camera = self.camera_matrix_inv @ head_position_homogeneous

            relative_pan_angle = -np.arctan(
                head_position_camera[0]
            )  # el menos hay que estudiarlo
            relative_tilt_angle = np.arctan(head_position_camera[1])

            self.last_face_detection_time = time.time()

            if time.time() - self.last_head_move > 1:
                self.node.get_logger().info(
                    "Cara detectada. Moviendo hacia ("
                    + str(format(self.pan + relative_pan_angle, ".2f"))
                    + ", "
                    + str(format(self.tilt + relative_tilt_angle, ".2f"))
                    + ")"
                )
                self.node.pub_neck_joint.publish(
                    JointGroupCommand(
                        name="turret",
                        cmd=[
                            self.pan + relative_pan_angle,
                            self.tilt + relative_tilt_angle,
                        ],
                    )
                )

                self.last_head_move = time.time()

    def move_with_sound(self, audio_intensity, sound_angle):
        if audio_intensity < self.audio_intensity_threshold:
            if (
                time.time() - self.last_sound_angle_time
                > self.max_delay_seconds_face + self.max_delay_seconds_sound
            ):  # NO hay angulo por más de 5 segundos
                self.node.pub_neck_joint.publish(
                    JointGroupCommand(name="turret", cmd=[0, 0])
                )
        else:
            self.node.get_logger().info(
                "Sin caras. Guiado por sonido. Intensidad: "
                + str(int(audio_intensity))
                + ". Angulo: "
                + str(format(sound_angle, ".2f"))
            )
            self.node.pub_neck_joint.publish(
                JointGroupCommand(name="turret", cmd=[sound_angle, 0])
            )

            self.last_sound_angle_time = time.time()

    def detection_request(self, frame_msg):
        detection_request = Detection.Request()
        detection_request.frame = frame_msg

        future_detection = self.node.detection_client.call_async(detection_request)
        rclpy.spin_until_future_complete(self.node, future_detection)
        result_detection = future_detection.result()

        return result_detection.positions, result_detection.scores

    def audio_location_request(self):
        audio_location_request = AudioLocation.Request()

        future_audio_location = self.node.audio_location_client.call_async(
            audio_location_request
        )
        rclpy.spin_until_future_complete(self.node, future_audio_location)
        result_audio_location = future_audio_location.result()

        return result_audio_location.intensity, result_audio_location.angle_rad

    def init_head(self):
        request = OperatingModes.Request()
        request.cmd_type = "group"
        request.name = "turret"
        request.mode = "position"
        request.profile_type = "time"
        request.profile_velocity = 1000
        request.profile_acceleration = 500

        future = self.node.serv_neck_op.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info(
                "Service call successful. Response: %s" % future.result()
            )
        else:
            self.node.get_logger().info("Service call failed")

        self.node.pub_neck_joint.publish(JointGroupCommand(name="turret", cmd=[0, 0]))

    def brightness(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        hist_flat = np.ravel(hist)
        total_pixels = sum(hist_flat)

        brightness = sum(hist_flat * np.arange(256)) / total_pixels

        return brightness


def main(args=None):
    rclpy.init(args=args)

    hri_head = HRIHead()
    hri_head.spin()

    rclpy.shutdown()
