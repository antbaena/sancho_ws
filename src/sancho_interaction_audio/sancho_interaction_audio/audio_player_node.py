import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sancho_msgs.action import PlayAudio
import asyncio

class AudioPlayer(LifecycleNode):
    def __init__(self):
        super().__init__('audio_player')

        # Action server will be created on activation
        self._action_server = None
        self._current_task = None

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('AudioPlayer ACTIVATED: starting action server')
        # Create action server
        self._action_server = ActionServer(
            self,
            PlayAudio,
            'play_audio',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('AudioPlayer DEACTIVATED: shutting down')
        # Cancel any playing audio
        if self._current_task and not self._current_task.done():
            self._current_task.cancel()
        # Destroy action server
        if self._action_server:
            self._action_server.destroy()
            self._action_server = None
        return super().on_deactivate(state)

    def goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info(f'Received request to play: {goal_request.filename}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def _play_audio(self, goal_handle):
        # Dummy implementation: simulate playback time from filename length
        duration = min(len(goal_handle.request.filename), 5)
        self.get_logger().info(f'Simulating playback for {duration}s')
        try:
            await asyncio.sleep(duration)
        except asyncio.CancelledError:
            self.get_logger().info('Playback cancelled')
            goal_handle.canceled()
            return
        # On success
        result = PlayAudio.Result()
        result.success = True
        result.message = 'Playback finished'
        goal_handle.succeed()
        goal_handle.set_result(result)

    def execute_callback(self, goal_handle):
        # Launch playback in background
        loop = asyncio.get_event_loop()
        self._current_task = loop.create_task(self._play_audio(goal_handle))
        return self._current_task


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
