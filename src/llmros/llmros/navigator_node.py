import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

from ros_llm_interfaces.srv import SaveLocation, GetLocation, DeleteLocation, ListLocations
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

import json
import threading
import time
from enum import Enum

class NavigatorState(Enum):
    IDLE = 0
    NAVIGATING = 1
    MANUAL_CONTROL = 2
    SAVING_LOCATION = 3
    DELETING_LOCATION = 4
    LISTING_LOCATIONS = 5
    ERROR = 6

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.get_logger().info('Navigator Node started.')

        # --- Parameters --- #
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('pose_freshness_threshold_sec', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_link_frame', 'base_link')

        self.cmd_vel_timeout_sec = self.get_parameter('cmd_vel_timeout_sec').get_parameter_value().double_value
        self.pose_freshness_threshold_sec = self.get_parameter('pose_freshness_threshold_sec').get_parameter_value().double_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value

        # --- State Management --- #
        self.current_state = NavigatorState.IDLE
        self._state_lock = threading.Lock()

        # --- Current Pose Tracking --- #
        self.current_pose_stamped = None  # PoseWithCovarianceStamped from /amcl_pose
        self.last_pose_time = self.get_clock().now()

        # --- TF Buffer and Listener --- #
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- QoS Profile --- #
        # For sensor data (like /amcl_pose), use reliable and keep last 1
        qos_profile_sensor_data = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # For command topics (like /cmd_vel), use best effort and keep last 1
        qos_profile_cmd_vel = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- ROS Subscriptions and Publishers --- #
        self.create_subscription(String, 'text_command', self._voice_cmd_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_pose_callback, qos_profile=qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile=qos_profile_cmd_vel)

        # --- Service Clients --- #
        self.save_loc_client = self.create_client(SaveLocation, 'save_location')
        self.get_loc_client = self.create_client(GetLocation, 'get_location')
        self.delete_loc_client = self.create_client(DeleteLocation, 'delete_location')
        self.list_loc_client = self.create_client(ListLocations, 'list_locations')

        # Wait for service servers to be available
        self.get_logger().info('Waiting for location manager services...')
        self.save_loc_client.wait_for_service()
        self.get_loc_client.wait_for_service()
        self.delete_loc_client.wait_for_service()
        self.list_loc_client.wait_for_service()
        self.get_logger().info('Location manager services available.')

        # --- Nav2 Action Client --- #
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self._nav_action_client.wait_for_server()
        self.get_logger().info('Nav2 action server available.')

        # --- Timers --- #
        self._cmd_vel_timer = self.create_timer(self.cmd_vel_timeout_sec / 2.0, self._cmd_vel_timeout_callback) # Check twice as fast as timeout
        self._last_cmd_vel_time = self.get_clock().now()

        # --- Initial Checks --- #
        self._check_tf_availability()

    def _set_state(self, new_state):
        with self._state_lock:
            if self.current_state != new_state:
                self.get_logger().info(f'State transition: {self.current_state.name} -> {new_state.name}')
                self.current_state = new_state

    def _check_tf_availability(self):
        self.get_logger().info(f'Checking TF transform from {self.base_link_frame} to {self.map_frame}...')
        try:
            # Wait for the transform to be available
            self.tf_buffer.lookup_transform(self.map_frame, self.base_link_frame, rclpy.time.Time(), Duration(seconds=5.0))
            self.get_logger().info('TF transform available. System ready.')
        except TransformException as ex:
            self.get_logger().error(f'Could not transform {self.base_link_frame} to {self.map_frame}: {ex}')
            self._set_state(NavigatorState.ERROR)

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose_stamped = msg
        self.last_pose_time = self.get_clock().now()

    def _publish_cmd_vel(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_publisher.publish(twist)
        self._last_cmd_vel_time = self.get_clock().now()

    def _cmd_vel_timeout_callback(self):
        # If no command has been published for a while, stop the robot
        if self.current_state == NavigatorState.MANUAL_CONTROL and \
           (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9 > self.cmd_vel_timeout_sec:
            self.get_logger().warn('cmd_vel timeout. Stopping robot.')
            self._publish_cmd_vel(0.0, 0.0) # Send zero velocity
            self._set_state(NavigatorState.IDLE) # Return to idle after timeout

    def _voice_cmd_callback(self, msg: String):
        self.get_logger().info(f'Received command from LLM: {msg.data}')
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('action')
            params = cmd.get('params', {})

            # Handle error action from LLMNode
            if action == 'error':
                error_msg = params.get('message', 'Unknown LLM error')
                self.get_logger().error(f'LLM Error: {error_msg}')
                self._send_feedback_to_user(f'LLM 오류: {error_msg}')
                self._set_state(NavigatorState.IDLE)
                return

            # State-based command handling
            with self._state_lock:
                if self.current_state == NavigatorState.NAVIGATING and action != 'cancel_navigation':
                    self.get_logger().warn(f'Ignoring command {action} while navigating. Please cancel current navigation first.')
                    self._send_feedback_to_user('현재 주행 중입니다. 먼저 주행을 취소해주세요.')
                    return
                if self.current_state == NavigatorState.MANUAL_CONTROL and action not in ['move', 'rotate']:
                    self.get_logger().warn(f'Ignoring command {action} while in manual control. Please wait or stop manual control.')
                    self._send_feedback_to_user('현재 수동 제어 중입니다. 다른 명령을 내리려면 잠시 기다려주세요.')
                    return

                # Dispatch based on action
                if action == 'move':
                    self._set_state(NavigatorState.MANUAL_CONTROL)
                    self._handle_move(params)
                elif action == 'rotate':
                    self._set_state(NavigatorState.MANUAL_CONTROL)
                    self._handle_rotate(params)
                elif action == 'save_location':
                    self._set_state(NavigatorState.SAVING_LOCATION)
                    self._handle_save_location(params)
                elif action == 'go_to_location':
                    self._set_state(NavigatorState.NAVIGATING)
                    self._handle_go_to_location(params)
                elif action == 'delete_location':
                    self._set_state(NavigatorState.DELETING_LOCATION)
                    self._handle_delete_location(params)
                elif action == 'list_locations':
                    self._set_state(NavigatorState.LISTING_LOCATIONS)
                    self._handle_list_locations(params)
                elif action == 'cancel_navigation': # For future use, if LLM can generate this
                    self._cancel_navigation()
                else:
                    self.get_logger().warn(f'Unsupported action: {action}')
                    self._send_feedback_to_user(f'지원하지 않는 명령입니다: {action}')
                    self._set_state(NavigatorState.IDLE)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON from /voice_cmd: {msg.data}. Error: {e}')
            self._send_feedback_to_user('명령어 파싱 오류가 발생했습니다.')
            self._set_state(NavigatorState.IDLE)
        except Exception as e:
            self.get_logger().error(f'Error in voice_cmd_callback: {e}')
            self._send_feedback_to_user(f'내부 오류 발생: {e}')
            self._set_state(NavigatorState.ERROR)

    # --- Action Handlers --- #

    def _handle_move(self, params):
        linear_speed = params.get('linear_speed', 0.2)
        # distance = params.get('distance', 0.0) # For future, if we want to move exact distance
        # is_forward = params.get('is_forward', True)
        self._publish_cmd_vel(linear_speed, 0.0)
        self._send_feedback_to_user(f'선형 속도 {linear_speed}로 이동합니다.')

    def _handle_rotate(self, params):
        angular_velocity = params.get('angular_velocity', 0.5) # rad/s or deg/s depending on LLM output
        is_clockwise = params.get('is_clockwise', True)
        # Convert degrees to radians if LLM outputs degrees
        # Assuming LLM outputs degrees for angular_velocity
        angular_velocity_rad = angular_velocity * (3.14159 / 180.0)
        if is_clockwise:
            angular_velocity_rad *= -1
        self._publish_cmd_vel(0.0, angular_velocity_rad)
        self._send_feedback_to_user(f'각속도 {angular_velocity}로 회전합니다.')

    def _handle_save_location(self, params):
        location_name = params.get('name')
        if not location_name:
            self.get_logger().error('Save location command missing name parameter.')
            self._send_feedback_to_user('장소 이름을 알려주세요.')
            self._set_state(NavigatorState.IDLE)
            return

        if self.current_pose_stamped is None or \
           (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9 > self.pose_freshness_threshold_sec:
            self.get_logger().warn('Cannot save location: Current pose is not fresh or available.')
            self._send_feedback_to_user('현재 위치 정보를 알 수 없거나 너무 오래되었습니다. 잠시 후 다시 시도해주세요.')
            self._set_state(NavigatorState.IDLE)
            return

        # Ensure pose is in map frame before saving
        if self.current_pose_stamped.header.frame_id != self.map_frame:
            self.get_logger().warn(f'Current pose is in {self.current_pose_stamped.header.frame_id} frame, not {self.map_frame}. Attempting transform.')
            try:
                transform = self.tf_buffer.lookup_transform(self.map_frame, self.current_pose_stamped.header.frame_id, self.current_pose_stamped.header.stamp, Duration(seconds=1.0))
                pose_in_map = do_transform_pose(self.current_pose_stamped.pose, transform)
                pose_to_save = PoseStamped(header=self.current_pose_stamped.header, pose=pose_in_map)
                pose_to_save.header.frame_id = self.map_frame # Ensure frame_id is map
            except TransformException as ex:
                self.get_logger().error(f'Could not transform pose to map frame: {ex}')
                self._send_feedback_to_user('현재 위치를 지도 프레임으로 변환할 수 없습니다.')
                self._set_state(NavigatorState.IDLE)
                return
        else:
            pose_to_save = PoseStamped(header=self.current_pose_stamped.header, pose=self.current_pose_stamped.pose)

        req = SaveLocation.Request()
        req.name = location_name
        req.pose = pose_to_save

        self.get_logger().info(f'Calling save_location service for {location_name}...')
        future = self.save_loc_client.call_async(req)
        future.add_done_callback(lambda future: self._save_location_response_callback(future, location_name))

    def _save_location_response_callback(self, future, location_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Location \'{location_name}\' saved successfully: {response.message}')
                self._send_feedback_to_user(f'장소 \'{location_name}\'이(가) 성공적으로 저장되었습니다.')
            else:
                self.get_logger().error(f'Failed to save location \'{location_name}\' : {response.message}')
                self._send_feedback_to_user(f'장소 \'{location_name}\' 저장에 실패했습니다: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self._send_feedback_to_user('장소 저장 서비스 호출 중 오류가 발생했습니다.')
        finally:
            self._set_state(NavigatorState.IDLE)

    def _handle_go_to_location(self, params):
        location_name = params.get('name')
        if not location_name:
            self.get_logger().error('Go to location command missing name parameter.')
            self._send_feedback_to_user('이동할 장소 이름을 알려주세요.')
            self._set_state(NavigatorState.IDLE)
            return

        self._cancel_navigation() # Cancel any ongoing navigation

        req = GetLocation.Request()
        req.name = location_name

        self.get_logger().info(f'Calling get_location service for {location_name}...')
        future = self.get_loc_client.call_async(req)
        future.add_done_callback(lambda future: self._get_location_response_callback(future, location_name))

    def _get_location_response_callback(self, future, location_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Retrieved location \'{location_name}\'. Sending to Nav2.')
                # Ensure the retrieved pose is in the map frame
                if response.pose.header.frame_id != self.map_frame:
                    self.get_logger().warn(f'Retrieved pose for {location_name} is not in {self.map_frame} frame. Attempting transform.')
                    try:
                        transform = self.tf_buffer.lookup_transform(self.map_frame, response.pose.header.frame_id, response.pose.header.stamp, Duration(seconds=1.0))
                        pose_in_map = do_transform_pose(response.pose.pose, transform)
                        goal_pose = PoseStamped(header=response.pose.header, pose=pose_in_map)
                        goal_pose.header.frame_id = self.map_frame # Ensure frame_id is map
                    except TransformException as ex:
                        self.get_logger().error(f'Could not transform retrieved pose to map frame: {ex}')
                        self._send_feedback_to_user('저장된 장소의 위치를 지도 프레임으로 변환할 수 없습니다.')
                        self._set_state(NavigatorState.IDLE)
                        return
                else:
                    goal_pose = response.pose

                self._send_nav_goal(goal_pose)
            else:
                self.get_logger().error(f'Failed to get location \'{location_name}\' : {response.message}')
                self._send_feedback_to_user(f'장소 \'{location_name}\'을(를) 찾을 수 없습니다.')
                self._set_state(NavigatorState.IDLE)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self._send_feedback_to_user('장소 조회 서비스 호출 중 오류가 발생했습니다.')
            self._set_state(NavigatorState.IDLE)

    def _handle_delete_location(self, params):
        location_name = params.get('name')
        if not location_name:
            self.get_logger().error('Delete location command missing name parameter.')
            self._send_feedback_to_user('삭제할 장소 이름을 알려주세요.')
            self._set_state(NavigatorState.IDLE)
            return

        req = DeleteLocation.Request()
        req.name = location_name

        self.get_logger().info(f'Calling delete_location service for {location_name}...')
        future = self.delete_loc_client.call_async(req)
        future.add_done_callback(lambda future: self._delete_location_response_callback(future, location_name))

    def _delete_location_response_callback(self, future, location_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Location \'{location_name}\' deleted successfully: {response.message}')
                self._send_feedback_to_user(f'장소 \'{location_name}\'이(가) 성공적으로 삭제되었습니다.')
            else:
                self.get_logger().error(f'Failed to delete location \'{location_name}\' : {response.message}')
                self._send_feedback_to_user(f'장소 \'{location_name}\' 삭제에 실패했습니다: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self._send_feedback_to_user('장소 삭제 서비스 호출 중 오류가 발생했습니다.')
        finally:
            self._set_state(NavigatorState.IDLE)

    def _handle_list_locations(self, params):
        req = ListLocations.Request()
        self.get_logger().info('Calling list_locations service...')
        future = self.list_loc_client.call_async(req)
        future.add_done_callback(self._list_locations_response_callback)

    def _list_locations_response_callback(self, future):
        try:
            response = future.result()
            if response.names:
                locations_str = ", ".join(response.names)
                self.get_logger().info(f'Available locations: {locations_str}')
                self._send_feedback_to_user(f'저장된 장소: {locations_str}')
            else:
                self.get_logger().info('No locations saved.')
                self._send_feedback_to_user('저장된 장소가 없습니다.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self._send_feedback_to_user('장소 목록 조회 서비스 호출 중 오류가 발생했습니다.')
        finally:
            self._set_state(NavigatorState.IDLE)

    # --- Nav2 Action Client Methods --- #

    def _send_nav_goal(self, pose: PoseStamped):
        self.get_logger().info(f'Sending navigation goal to: {pose.pose.position.x}, {pose.pose.position.y}')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._nav_action_client.wait_for_server()
        self._send_goal_future = self._nav_action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 action server.')
            self._send_feedback_to_user('주행 목표가 거부되었습니다.')
            self._set_state(NavigatorState.IDLE)
            return

        self.get_logger().info('Goal accepted by Nav2 action server.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}')
        # You can add more detailed feedback to the user here if needed

    def _get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == ActionClient.GoalStatus.SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            self._send_feedback_to_user('목표 지점에 성공적으로 도착했습니다.')
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            self._send_feedback_to_user(f'주행에 실패했습니다. 상태 코드: {status}')
        
        self._set_state(NavigatorState.IDLE)

    def _cancel_navigation(self):
        self.get_logger().info('Cancelling current navigation goal...')
        # Check if there's an active goal to cancel
        if hasattr(self, '_send_goal_future') and not self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if goal_handle and goal_handle.accepted:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_callback)
            else:
                self.get_logger().info('No active accepted goal to cancel.')
                self._set_state(NavigatorState.IDLE)
        else:
            self.get_logger().info('No active goal future to cancel.')
            self._set_state(NavigatorState.IDLE)

    def _cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Navigation goal successfully cancelled.')
            self._send_feedback_to_user('주행이 취소되었습니다.')
        else:
            self.get_logger().warn('Failed to cancel navigation goal.')
            self._send_feedback_to_user('주행 취소에 실패했습니다.')
        self._set_state(NavigatorState.IDLE)

    def _send_feedback_to_user(self, message):
        # This method can be expanded to publish to a user feedback topic
        # or integrate with a text-to-speech system.
        self.get_logger().info(f'USER FEEDBACK: {message}')

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()