import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from person_tracker_msgs.msg import PersonFeature, PersonsFeatureArray
import numpy as np
from scipy.optimize import linear_sum_assignment
from filterpy.kalman import KalmanFilter
from collections import deque
import tf2_ros

class Track:
    def __init__(self, track_id, initial_pos, initial_emb, kf_config, history_len):
        self.id = track_id
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        dt = kf_config['dt']
        self.kf.F = np.array([[1, 0, dt, 0],
                              [0, 1, 0, dt],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]])
        self.kf.P *= kf_config['P0']
        self.kf.R = np.eye(2) * kf_config['R']
        self.kf.Q = np.eye(4) * kf_config['Q']
        self.kf.x[:2] = np.array(initial_pos).reshape((2,1))
        self.kf.x[2:] = 0.0
        self.emb_history = deque([initial_emb], maxlen=history_len)
        self.missed = 0

    def predict(self):
        self.kf.predict()

    def update(self, pos, emb):
        self.kf.update(np.array(pos))
        self.emb_history.append(emb)
        self.missed = 0

    @property
    def current_position(self):
        return self.kf.x[:2].flatten()

    @property
    def average_embedding(self):
        return np.mean(np.stack(self.emb_history), axis=0)

class PersonTrackerKFNode(LifecycleNode):
    def __init__(self):
        super().__init__('person_tracker_kf')
        # Parameters
        self.declare_parameter('motion_weight', 0.5)
        self.declare_parameter('appearance_weight', 0.5)
        self.declare_parameter('cost_threshold', 1.0)
        self.declare_parameter('max_missed', 5)
        self.declare_parameter('history_len', 10)
        self.declare_parameter('kf_dt', 0.1)
        self.declare_parameter('kf_P0', 1e3)
        self.declare_parameter('kf_R', 0.1)
        self.declare_parameter('kf_Q', 0.01)
        self.declare_parameter('selection_mode', 'nearest')  # 'nearest', 'first_seen', or 'manual'
        self.declare_parameter('target_id', -1)
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('world_frame', 'map')

        self.tracks = {}
        self.next_id = 0

        # Loaded parameter values
        self.motion_weight = None
        self.appearance_weight = None
        self.cost_threshold = None
        self.max_missed = None
        self.history_len = None
        self.kf_config = {}
        self.selection_mode = None
        self.target_id = None
        self.robot_frame = None
        self.world_frame = None

        # TF
        self.tf_buffer = None
        self.tf_listener = None

        # ROS interfaces
        self.sub = None
        self.marker_pub = None
        self.follow_pub = None

    def on_configure(self, state):
        # Read parameters
        self.motion_weight = self.get_parameter('motion_weight').value
        self.appearance_weight = self.get_parameter('appearance_weight').value
        self.cost_threshold = self.get_parameter('cost_threshold').value
        self.max_missed = self.get_parameter('max_missed').value
        self.history_len = self.get_parameter('history_len').value
        self.kf_config = {
            'dt': self.get_parameter('kf_dt').value,
            'P0': self.get_parameter('kf_P0').value,
            'R': self.get_parameter('kf_R').value,
            'Q': self.get_parameter('kf_Q').value
        }
        self.selection_mode = self.get_parameter('selection_mode').value
        self.target_id = self.get_parameter('target_id').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.world_frame = self.get_parameter('world_frame').value

        qos = QoSProfile(depth=10)
        self.sub = self.create_lifecycle_subscription(
            PersonsFeatureArray,
            '/human_pose/person_features',
            self.feature_callback,
            qos)
        self.marker_pub = self.create_lifecycle_publisher(
            MarkerArray, '/person_marker_array', qos)
        self.follow_pub = self.create_lifecycle_publisher(
            PoseStamped, '/follow_target', qos)
        # Setup TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('Configured KF tracker with parameters')
        return super().on_configure(state)

    def on_activate(self, state):
        self.marker_pub.on_activate()
        self.follow_pub.on_activate()
        self.get_logger().info('Activated KF tracker')
        return super().on_activate(state)

    def feature_callback(self, msg: PersonsFeatureArray):
        # Predict all tracks
        for tr in self.tracks.values():
            tr.predict()

        # Gather detections
        det_pos = []
        det_emb = []
        det_ids = []
        for feat in msg.features:
            det_ids.append(feat.id)
            det_pos.append([feat.position.point.x, feat.position.point.y])
            det_emb.append(np.array(feat.embedding))
        det_pos = np.array(det_pos)

        # Selection logic
        # Robot pose
        try:
            t = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.robot_frame,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1))
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
        except Exception:
            robot_x = robot_y = 0.0

        if self.selection_mode == 'nearest' and det_pos.size:
            dists = [np.linalg.norm(p - np.array([robot_x, robot_y])) for p in det_pos]
            idx = int(np.argmin(dists))
            self.target_id = det_ids[idx]
        elif self.selection_mode == 'first_seen' and self.target_id < 0:
            if det_ids:
                self.target_id = det_ids[0]
        # 'manual' leaves target_id unchanged

        # Build cost matrix
        track_list = list(self.tracks.values())
        N, M = len(track_list), len(det_pos)
        cost = np.zeros((N, M))
        for i, tr in enumerate(track_list):
            for j in range(M):
                motion_dist = np.linalg.norm(tr.current_position - det_pos[j])
                emb_dist = 1 - np.dot(tr.average_embedding, det_emb[j]) / (
                    np.linalg.norm(tr.average_embedding) * np.linalg.norm(det_emb[j]) + 1e-6)
                cost[i, j] = self.motion_weight * motion_dist + self.appearance_weight * emb_dist

        # Assignment
        if N and M:
            row_ind, col_ind = linear_sum_assignment(cost)
        else:
            row_ind, col_ind = np.array([], int), np.array([], int)

        matched_tracks, matched_dets = set(), set()
        for r, c in zip(row_ind, col_ind):
            if cost[r, c] < self.cost_threshold:
                tr = track_list[r]
                tr.update(det_pos[c], det_emb[c])
                matched_tracks.add(tr.id)
                matched_dets.add(c)

        # Manage missed
        for tr in track_list:
            if tr.id not in matched_tracks:
                tr.missed += 1
                if tr.missed > self.max_missed:
                    del self.tracks[tr.id]

        # Create new
        for j in range(M):
            if j not in matched_dets:
                new_tr = Track(
                    self.next_id,
                    det_pos[j],
                    det_emb[j],
                    self.kf_config,
                    self.history_len)
                self.tracks[self.next_id] = new_tr
                self.next_id += 1

        # Publish markers
        m_arr = MarkerArray()
        for tr in self.tracks.values():
            m = Marker(header=msg.header,
                       ns='person_tracker',
                       id=tr.id,
                       type=Marker.SPHERE,
                       action=Marker.ADD)
            m.pose.position.x = tr.current_position[0]
            m.pose.position.y = tr.current_position[1]
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.a = 1.0
            m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
            m_arr.markers.append(m)
        self.marker_pub.publish(m_arr)

        # Publish follow target
        if self.target_id in self.tracks:
            tr = self.tracks[self.target_id]
            goal = PoseStamped()
            goal.header = msg.header
            goal.pose.position.x = tr.current_position[0]
            goal.pose.position.y = tr.current_position[1]
            goal.pose.orientation.w = 1.0
            self.follow_pub.publish(goal)

    def on_deactivate(self, state):
        self.get_logger().info('Deactivated KF tracker')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info('Cleaned up KF tracker')
        return super().on_cleanup(state)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
