import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from geometry_msgs.msg import Twist
from sancho_msgs.msg import FaceArray
from sancho_msgs.action import PlayAudio
from std_msgs.msg import Float32
from message_filters import Subscriber
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.task import Future

class InteractionManager(LifecycleNode):

    # Estados de la state machine
    (
        STATE_IDLE,
        STATE_ACTIVATE_FACE_DETECTOR,
        STATE_SEARCH_FACE,
        STATE_FALLBACK_TDOA,
        STATE_TRACK_AND_AUDIO,
        STATE_DEACTIVATE_ALL,
        STATE_DONE
    ) = range(7)

    def __init__(self):
        super().__init__('interaction_manager_sync')
        # Callback groups para separar lifecycle / IO
        self.life_cb = ReentrantCallbackGroup()
        self.io_cb   = ReentrantCallbackGroup()

        # Parámetros
        self.declare_parameter('face_size_threshold', 0.1)
        self.declare_parameter('face_confidence_threshold', 0.7)
        self.declare_parameter('max_face_attempts', 3)
        self.declare_parameter('tdoa_angle_limit', 90.0)
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('rotation_duration', 1.0)

        # Módulos a gestionar
        self.modules = ['/face_detector', '/face_tracker', '/audio_player']
        self.module_clients = {}

        # QoS
        self.sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.control_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions y publishers (inactivas al principio)
        self.face_sub = None
        self.tdoa_sub = None
        self.cmd_vel = None

        # Action client
        self.audio_action = None

        # Variables de runtime
        self.face_msgs = []
        self.tdoa_angle = None
        self.attempt = 0
        self.best_face = None

        # Máquina de estados
        self.state = self.STATE_IDLE
        self.main_timer = None

        self.get_logger().info('InteractionManager (sync) creado')

    # ------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------
    def on_configure(self, state):
        self.get_logger().info('Configuring InteractionManager')

        # Leer parámetros
        self.max_attempts = self.get_parameter('max_face_attempts').value
        self.face_size_threshold = self.get_parameter('face_size_threshold').value
        self.face_confidence_threshold = self.get_parameter('face_confidence_threshold').value
        self.tdoa_angle_limit = self.get_parameter('tdoa_angle_limit').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.rotation_duration = self.get_parameter('rotation_duration').value

        # Crear clients de ciclo de vida para cada módulo
        for m in self.modules:
            cli = self.create_client(ChangeState, f'{m}/change_state',
                                     callback_group=self.life_cb)
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"No se encontró el servicio {m}/change_state")
                return TransitionCallbackReturn.FAILURE
            self.module_clients[m] = cli
            self.get_logger().info(f"Cliente de lifecycle para {m} listo")

        # Publisher de cmd_vel (lifecycle)
        self.cmd_vel = self.create_lifecycle_publisher(
            Twist, 'cmd_vel', self.control_qos)

        # Action client para audio
        self.audio_action = ActionClient(
            self, PlayAudio, 'play_audio', callback_group=self.life_cb)
        if not self.audio_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor de action 'play_audio' no disponible")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('InteractionManager configurado')
        return super().on_configure(state)

    def on_activate(self, state):
        self.get_logger().info('Activando interacción (sync)')
        # Subscriptions de sensores
        self.face_sub = Subscriber(self, FaceArray, 'face_detections',
                                   qos_profile=self.sensor_qos,
                                   callback_group=self.io_cb)
        self.face_sub.registerCallback(self._face_cb)

        self.tdoa_sub = Subscriber(self, Float32, 'tdoa_angle',
                                   qos_profile=self.sensor_qos,
                                   callback_group=self.io_cb)
        self.tdoa_sub.registerCallback(self._tdoa_cb)

        # Iniciar máquina de estados en primer estado útil
        self.state = self.STATE_ACTIVATE_FACE_DETECTOR
        self.main_timer = self.create_timer(0.1, self._run_state)
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('Desactivando interacción (sync)')
        if self.main_timer:
            self.main_timer.cancel()
            self.main_timer = None
        # Destruir subscriptions
        if self.face_sub:
            self.destroy_subscription(self.face_sub)
            self.face_sub = None
        if self.tdoa_sub:
            self.destroy_subscription(self.tdoa_sub)
            self.tdoa_sub = None
        # Publisher y clients
        if self.cmd_vel:
            self.destroy_lifecycle_publisher(self.cmd_vel)
            self.cmd_vel = None
        for cli in self.module_clients.values():
            self.destroy_client(cli)
        self.module_clients.clear()
        return super().on_deactivate(state)

    # ------------------------------------------------------------
    # Callbacks de sensores
    # ------------------------------------------------------------
    def _face_cb(self, msg: FaceArray):
        self.face_msgs = msg.faces

    def _tdoa_cb(self, msg: Float32):
        self.tdoa_angle = msg.data

    # ------------------------------------------------------------
    # Utils sync: llamar servicio de lifecycle
    # ------------------------------------------------------------
    def call_lifecycle(self, module: str, transition_id: int) -> bool:
        cli = self.module_clients.get(module)
        if cli is None:
            self.get_logger().error(f"Cliente inexistente: {module}")
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().success:
            self.get_logger().error(f"{module} transición {transition_id} fallida")
            return False
        return True

    # ------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------
    def _run_state(self):
        if self.state == self.STATE_ACTIVATE_FACE_DETECTOR:
            if self.call_lifecycle('/face_detector', Transition.TRANSITION_ACTIVATE):
                self.get_logger().info('Face detector activado')
                self.attempt = 0
                self.state = self.STATE_SEARCH_FACE
            else:
                self.state = self.STATE_DEACTIVATE_ALL

        elif self.state == self.STATE_SEARCH_FACE:
            self.best_face = self._select_best_face()
            if self.best_face:
                self.get_logger().info('Cara encontrada!')
                self.state = self.STATE_TRACK_AND_AUDIO
            elif self.attempt < self.max_attempts:
                # girar base y reintentar
                # direction = self.rotation_speed * self.rotation_duration
                # if self.attempt % 2 == 1:
                #     direction = -direction
                # self.get_logger().info(f"Intento {self.attempt+1}, girando {'izq' if direction>0 else 'der'}")
                # self._rotate_base(direction)
                self.get_logger().info(f"DUMMY: Intento {self.attempt+1}, girando CABEZA")
                self.attempt += 1
            else:
                self.get_logger().warn('No se encontró cara, fallback TDOA')
                # self.state = self.STATE_FALLBACK_TDOA
                self.state = self.STATE_DEACTIVATE_ALL #dummy

        elif self.state == self.STATE_FALLBACK_TDOA:
            if abs(self.tdoa_angle or 999) <= self.tdoa_angle_limit:
                self.get_logger().info(f"Girando por TDOA: {self.tdoa_angle:.1f}°")
                self._rotate_base(self.tdoa_angle)
                self.best_face = self._select_best_face()
                if self.best_face:
                    self.state = self.STATE_TRACK_AND_AUDIO
                    return
            self.state = self.STATE_DEACTIVATE_ALL

        elif self.state == self.STATE_TRACK_AND_AUDIO:
            self.call_lifecycle('/face_tracker', Transition.TRANSITION_ACTIVATE)
            self.call_lifecycle('/audio_player', Transition.TRANSITION_ACTIVATE)
            # Reproducir audio
            goal = PlayAudio.Goal()
            goal.filename = '/home/mapir/sancho_ws/audio_tts_xtts.wav'
            send_goal_fut = self.audio_action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_fut)
            result_fut = send_goal_fut.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_fut)
            self.get_logger().info('Audio reproducido, completando interacción')
            self.state = self.STATE_DEACTIVATE_ALL

        elif self.state == self.STATE_DEACTIVATE_ALL:
            # Desactivamos todos los módulos y luego el nodo
            for m in self.modules:
                self.call_lifecycle(m, Transition.TRANSITION_DEACTIVATE)
            self.trigger_transition(Transition.TRANSITION_DEACTIVATE)
            self.state = self.STATE_DONE

        # STATE_DONE o cualquier otro: nada más que hacer

    # ------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------
    def _select_best_face(self):
        best = None
        for f in self.face_msgs:
            if (f.confidence >= self.face_confidence_threshold
                and f.width >= self.face_size_threshold):
                if best is None or f.confidence > best.confidence:
                    best = f
        return best

    def _rotate_base(self, angle: float):
        twist = Twist()
        speed = self.rotation_speed
        twist.angular.z = speed if angle > 0 else -speed
        self.cmd_vel.publish(twist)
        # bloqueante: esperamos el tiempo necesario
        rclpy.sleep(Duration(seconds=abs(angle)/speed))
        twist.angular.z = 0.0
        self.cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = InteractionManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
