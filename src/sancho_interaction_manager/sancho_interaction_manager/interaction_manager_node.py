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
import asyncio

class InteractionManager(LifecycleNode):
    def __init__(self):
        super().__init__('interaction_manager')
        # Callback groups
        self.life_cb = ReentrantCallbackGroup()
        self.io_cb   = ReentrantCallbackGroup()

        # Parámetros configurables
        self.declare_parameter('face_size_threshold', 0.1)
        self.declare_parameter('face_confidence_threshold', 0.7)
        self.declare_parameter('max_face_attempts', 3)
        self.declare_parameter('tdoa_angle_limit', 90.0)
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('rotation_duration', 1.0)

        # Módulos a gestionar
        self.modules = ['/face_detector_lifecycle', 
                       '/face_tracker', 
                       '/audio_player', 
                       '/tdoa_processor']

        # QoS para sensores y control
        self.sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.control_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Variables internas
        self.modules_clients = {}
        self.cmd_vel = None
        self.audio_action = None
        self.face_sub = None
        self.tdoa_sub = None
        self.ats = None
        self.face_msgs = []
        self.tdoa_angle = None
        self.main_timer = None

        self._face_event = asyncio.Event()
        
        

    def on_configure(self, state):
        self.get_logger().info('Configuring InteractionManager')

        self.max_attempts = self.get_parameter('max_face_attempts').value
        self.face_size_threshold = self.get_parameter('face_size_threshold').value
        self.face_confidence_threshold = self.get_parameter('face_confidence_threshold').value
        self.tdoa_angle_limit = self.get_parameter('tdoa_angle_limit').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.rotation_duration = self.get_parameter('rotation_duration').value

        # Configura clientes a servicios de ciclo de vida de módulos
        for m in self.modules:
            self.get_logger().info(f"Configuring lifecycle client for {m}")
            # Crea cliente de ciclo de vida
            self.modules_clients[m] = self.create_client(
                ChangeState, f'{m}/change_state', callback_group=self.life_cb)
            # Espera a que el servicio esté disponible
            while not self.modules_clients[m].wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"Service {m}/change_state not available, waiting...")
            self.get_logger().info(f"Service {m}/change_state available")
        self.get_logger().info('Lifecycle clients configured')
        
        # Publisher de velocidad usando ciclo de vida
        self.cmd_vel = self.create_lifecycle_publisher(
            Twist, 'cmd_vel', self.control_qos)

        # Cliente de acción para reproducir audio
        self.audio_action = ActionClient(
            self, PlayAudio, 'play_audio', callback_group=self.life_cb)

        # Callback para parámetros dinámicos
        self.add_on_set_parameters_callback(self._on_param_event)

        self.get_logger().info('InteractionManager configured')
        return super().on_configure(state)

    def on_activate(self, state):
        # Inicializa suscripciones sincronizadas de sensores
        self.face_sub = Subscriber(
            self, FaceArray, 'face_detections', qos_profile=self.sensor_qos,
            callback_group=self.io_cb)
        self.face_sub.registerCallback(self._face_cb)

        self.tdoa_sub = Subscriber(
            self, Float32, 'tdoa_angle', qos_profile=self.sensor_qos,
            callback_group=self.io_cb)
        self.tdoa_sub.registerCallback(self._tdoa_cb)

        # self.ats = ApproximateTimeSynchronizer(
        #     [self.face_sub, self.tdoa_sub], queue_size=10, slop=0.1)
        # self.ats.registerCallback(self._synced_cb)

        # Lanza la rutina de interacción 
        self.get_logger().info('Starting interaction routine')
        self.main_timer = self.create_timer(1.0, self._run_interaction)

        self.get_logger().info('InteractionManager activated')
        return super().on_activate(state)
    def on_deactivate(self, state):
        # Limpia suscripciones y sincronizador
        # if self.ats:
        #     self.ats.callbacks.clear()
        #     self.ats = None
        if self.face_sub:
            self.destroy_subscription(self.face_sub)
            self.face_sub = None
        if self.tdoa_sub:
            self.destroy_subscription(self.tdoa_sub)
            self.tdoa_sub = None
        # Destruir publisher de cmd_vel
        if self.cmd_vel is not None:
            self.destroy_lifecycle_publisher(self.cmd_vel)
            self.cmd_vel = None
        # Destruir clientes de ciclo de vida
        for m in self.modules_clients:
            if m in self.modules_clients:
                self.destroy_client(self.modules_clients[m])
                del self.modules_clients[m]
        if self.main_timer:
            self.main_timer.cancel()
            self.main_timer = None
        self.get_logger().info('InteractionManager deactivated')
        return super().on_deactivate(state)

    def _on_param_event(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name in [
                'face_size_threshold', 'face_confidence_threshold',
                'max_face_attempts', 'tdoa_angle_limit',
                'rotation_speed', 'rotation_duration']:
                self.get_logger().info(f"Parameter {p.name} updated to {p.value}")
        return SetParametersResult(successful=True)

    def _synced_cb(self, face_msg: FaceArray, tdoa_msg: Float32):
        self.face_msgs = face_msg.faces
        self.tdoa_angle = tdoa_msg.data
        if self.select_best_face():
            self._face_event.set()
   
    def _face_cb(self, face_msg: FaceArray):
        self.face_msgs = face_msg.faces
        if self.select_best_face():
            self._face_event.set()

    def _tdoa_cb(self, tdoa_msg):
        pass
    
    def select_best_face(self):
        best = None
        for f in self.face_msgs:
            if f.confidence >= self.face_confidence_threshold and f.width >= self.face_size_threshold:
                if best is None or f.confidence > best.confidence:
                    best = f
        return best

    async def call_lifecycle(self, module: str, transition_id: int):
        client = self.modules_clients.get(module)
        if not client or not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {module}/change_state unavailable")
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        resp = await client.call_async(req)
        if not resp.success:
            self.get_logger().error(f"{module}: transition {transition_id} failed")
        return resp.success

    async def rotate_base(self, angle: float):
        twist = Twist()
        speed = self.get_parameter('rotation_speed').value
        twist.angular.z = speed if angle > 0 else -speed
        self.cmd_vel.publish(twist)
        await asyncio.sleep(abs(angle) / speed)
        twist.angular.z = 0.0
        self.cmd_vel.publish(twist)

    async def _run_interaction(self):
        # 1. Activa detección de caras
        if not await self.call_lifecycle('/face_detector_lifecycle', Transition.TRANSITION_ACTIVATE):
            return await self.deactivate()

        # 2. Búsqueda activa de cara con reintentos

        found = None
        for i in range(self.max_attempts):
            self._face_event.clear()
            try:
                await asyncio.wait_for(self._face_event.wait(), timeout=1.0)
                found = self.select_best_face()
                break
            except asyncio.TimeoutError:
                pass
                angle = self.rotation_speed * self.rotation_duration
                direction = angle if i % 2 == 0 else -angle
                self.get_logger().info(f"Attempt {i+1}, rotating {'left' if direction>0 else 'right'}")
                await self.rotate_base(direction)

        # 3. Fallback TDOA si no encontró cara
        if not found:
            return await self.deactivate()
            if not await self.call_lifecycle('tdoa_processor', Transition.TRANSITION_ACTIVATE):
                return await self.deactivate()
            await asyncio.sleep(0.5)
            ang = self.tdoa_angle
            lim = self.tdoa_angle_limit
            if ang is not None and abs(ang) <= lim:
                self.get_logger().info(f"Rotating by TDOA: {ang}")
                await self.rotate_base(ang)
                await asyncio.sleep(1.0)
                found = self.select_best_face()
            if not found:
                self.get_logger().warn('No face found after TDOA')
                return await self.deactivate()

        # 4. Tracking y audio
        await self.call_lifecycle('face_tracker', Transition.TRANSITION_ACTIVATE)
        await self.call_lifecycle('audio_player', Transition.TRANSITION_ACTIVATE)
        goal = PlayAudio.Goal()
        goal.filename = 'welcome_message.wav'
        action_fut = await self.audio_action.send_goal_async(goal)
        await action_fut.get_result_async()

        # 5. Finaliza secuencia y desactiva el propio nodo
        self.get_logger().info('Interaction complete, deactivating node')
        await self.deactivate()

    async def deactivate(self):
        # Desactiva todos los módulos y hace la transición del nodo
        for m in self.modules:
            await self.call_lifecycle(m, Transition.TRANSITION_DEACTIVATE)
        # Solicita la desactivación del nodo
        self.trigger_transition(Transition.TRANSITION_DEACTIVATE)

    



def main(args=None):
    rclpy.init(args=args)
    node = InteractionManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()