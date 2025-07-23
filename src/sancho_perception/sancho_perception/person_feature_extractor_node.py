import rclpy
from rclpy.lifecycle import LifecycleNode
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import message_filters
import cv2
import numpy as np
import torch
import tf2_ros
import tf2_geometry_msgs
from torchreid.utils import FeatureExtractor
import torchvision.transforms as T
from geometry_msgs.msg import PointStamped

# Importar los nuevos mensajes
from sancho_msgs.msg import PersonFeature, PersonsFeatureArray
from sancho_msgs.msg import PersonsPoses, PersonPose  # mensaje de entrada con array persons

class PersonTrackerLifecycle(LifecycleNode):
    def __init__(self):
        super().__init__('person_tracker_lifecycle')
        # Parámetros de extracción
        self.declare_parameter('margin_px', 30)
        self.declare_parameter('reid_model', 'osnet_x1_0')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('tracking_frame', 'base_link')

        # Subscribers sincronizados
        self.image_sub = None
        self.kp3d_sub = None
        self.ts = None

        # Publisher de features custom
        self.feature_pub = None

        # Extractor de características (Torchreid)
        self.extractor = None
        self.transform = None

        # TF2
        self.tf_buffer = None
        self.tf_listener = None

    def on_configure(self, state):
        # Inicializar extractor Torchreid
        model_name = self.get_parameter('reid_model').value
        device = self.get_parameter('device').value
        self.extractor = FeatureExtractor(
            model_name=model_name,
            model_path=None,
            device=device
        )
        # Transformaciones de imagen
        self.transform = T.Compose([
            T.ToTensor(),
            T.Resize((256, 128)),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        # Publisher de PersonsFeatureArray
        self.feature_pub = self.create_lifecycle_publisher(
            PersonsFeatureArray,
            '/human_pose/person_features', 10)
        self.get_logger().info('Configurado extractor de características')
        return super().on_configure(state)

    def on_activate(self, state):
        # Inicializar TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Subscripciones sincronizadas
        self.image_sub = message_filters.Subscriber(self, Image, '/astra_camera/camera/color/image_raw')
        self.kp3d_sub = message_filters.Subscriber(self, PersonsPoses, '/human_pose/keypoints3d')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.kp3d_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)
        self.get_logger().info('Activado extractor de características')
        return super().on_activate(state)

    def callback(self, img_msg: Image, kp_msg: PersonsPoses):
        # Convertir ROS Image a OpenCV BGR
        frame = self._ros_img_to_cv2(img_msg)
        margin = self.get_parameter('margin_px').value
        world_frame = self.get_parameter('tracking_frame').value

        arr = PersonsFeatureArray()
        arr.header = kp_msg.header

        for person in kp_msg.persons:
            # 1) Calcular centroid 3D excluyendo outliers
            pts3d = np.array([[p.x, p.y, p.z] for p in person.keypoints3d], dtype=np.float32)

            # Extraer scores como array plano de floats
            scores = np.array([s for s in person.scores], dtype=np.float32)
            
            valid = scores > 0.2
            pts3d = pts3d[valid]
            if len(pts3d) < 3:
                self.get_logger().debug(f'Persona {person.id}: pocos puntos 3D')
                continue
            # quitar outliers: distancia al punto mediano
            med = np.median(pts3d, axis=0)
            dists = np.linalg.norm(pts3d - med, axis=1)
            thresh = np.median(dists) * 2.0
            inliers = pts3d[dists < thresh]
            centroid = inliers.mean(axis=0)
            # Transformar a frame map
            p_cam = PointStamped()
            p_cam.header = kp_msg.header
            p_cam.point.x, p_cam.point.y, p_cam.point.z = centroid.tolist()
            try:
                p_map = self.tf_buffer.transform(p_cam, world_frame, timeout=rclpy.duration.Duration(seconds=0.1))
            except Exception as e:
                self.get_logger().warn(f'TF no disponible: {e}')
                continue

            # 2) Generar crop con keypoints 2D
            keypoints2d = [(int(pt.x), int(pt.y)) for pt in person.keypoints]
            try:
                rgba = extract_person_from_keypoints(frame, keypoints2d, inflation_radius=margin)
            except ValueError:
                self.get_logger().debug(f'Persona {person.id}: insufficient keypoints 2D')
                continue
            bgr_crop = rgba[..., :3]

            # 3) Embedding
            inp = scale_and_pad(bgr_crop, target_size=(256,128))
            inp_tensor = self.transform(inp).unsqueeze(0).to(self.extractor.device)
            with torch.no_grad():
                feat = self.extractor(inp_tensor)
            emb = feat.squeeze(0).cpu().numpy().tolist()

            # 4) Rellenar PersonFeature
            pf = PersonFeature()
            pf.id = person.id
            pf.embedding = emb
            pf.position = p_map
            arr.features.append(pf)

        if arr.features:
            self.feature_pub.publish(arr)
        self.get_logger().info(f'Publicadas {len(arr.features)} características de personas')

    def on_deactivate(self, state):
        self.ts = None
        self.get_logger().info('Desactivado extractor de características')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.extractor = None
        self.get_logger().info('Limpiado extractor de características')
        return super().on_cleanup(state)

    @staticmethod
    def _ros_img_to_cv2(img_msg: Image) -> np.ndarray:
        b_arr = np.frombuffer(img_msg.data, np.uint8)
        img = b_arr.reshape((img_msg.height, img_msg.width, -1))
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

# Funciones auxiliares fuera de la clase

def extract_person_from_keypoints(image, keypoints, inflation_radius=30):
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    valid_points = np.array([pt for pt in keypoints if pt is not None], dtype=np.int32)
    if len(valid_points) < 3:
        raise ValueError("Se necesitan al menos 3 keypoints válidos para formar una región.")
    hull = cv2.convexHull(valid_points)
    cv2.fillConvexPoly(mask, hull, 255)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (inflation_radius, inflation_radius))
    mask = cv2.dilate(mask, kernel, iterations=1)
    person = cv2.bitwise_and(image, image, mask=mask)
    bgr = person
    alpha = mask
    rgba = cv2.merge([bgr[..., 0], bgr[..., 1], bgr[..., 2], alpha])
    return rgba


def scale_and_pad(image, target_size=(256,128)):
    h, w = image.shape[:2]
    tgt_h, tgt_w = target_size
    scale = min(tgt_w / w, tgt_h / h)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(image, (new_w, new_h))
    pad_w = tgt_w - new_w
    pad_h = tgt_h - new_h
    top = pad_h // 2
    bottom = pad_h - top
    left = pad_w // 2
    right = pad_w - left
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right,
                                borderType=cv2.BORDER_CONSTANT,
                                value=[0, 0, 0])
    return padded


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerLifecycle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
