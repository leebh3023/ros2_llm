
import rclpy
from rclpy.node import Node
import json
import os
import threading
from geometry_msgs.msg import PoseStamped

from ros_llm_interfaces.srv import SaveLocation
from ros_llm_interfaces.srv import GetLocation
from ros_llm_interfaces.srv import DeleteLocation
from ros_llm_interfaces.srv import ListLocations

# Helper function to convert PoseStamped message to dictionary
def pose_stamped_to_dict(pose_stamped_msg):
    return {
        'header': {
            'stamp': {'sec': pose_stamped_msg.header.stamp.sec, 'nanosec': pose_stamped_msg.header.stamp.nanosec},
            'frame_id': pose_stamped_msg.header.frame_id
        },
        'pose': {
            'position': {'x': pose_stamped_msg.pose.position.x, 'y': pose_stamped_msg.pose.position.y, 'z': pose_stamped_msg.pose.position.z},
            'orientation': {'x': pose_stamped_msg.pose.orientation.x, 'y': pose_stamped_msg.pose.orientation.y, 'z': pose_stamped_msg.pose.orientation.z, 'w': pose_stamped_msg.pose.orientation.w}
        }
    }

# Helper function to convert dictionary to PoseStamped message
def dict_to_pose_stamped(pose_dict):
    ps = PoseStamped()
    ps.header.stamp.sec = pose_dict['header']['stamp']['sec']
    ps.header.stamp.nanosec = pose_dict['header']['stamp']['nanosec']
    ps.header.frame_id = pose_dict['header']['frame_id']
    ps.pose.position.x = pose_dict['pose']['position']['x']
    ps.pose.position.y = pose_dict['pose']['position']['y']
    ps.pose.position.z = pose_dict['pose']['position']['z']
    ps.pose.orientation.x = pose_dict['pose']['orientation']['x']
    ps.pose.orientation.y = pose_dict['pose']['orientation']['y']
    ps.pose.orientation.z = pose_dict['pose']['orientation']['z']
    ps.pose.orientation.w = pose_dict['pose']['orientation']['w']
    return ps

class LocationManager(Node):
    def __init__(self):
        super().__init__('location_manager')
        self.get_logger().info('Location Manager Node started.')

        # Declare parameters
        self.declare_parameter('locations_file_path', 'locations.json')
        
        # Get parameters
        file_path = self.get_parameter('locations_file_path').get_parameter_value().string_value
        
        # If the path is not absolute, assume it's relative to the package share directory
        if not os.path.isabs(file_path):
             self.get_logger().warn(f"Path '{file_path}' is not absolute. This is not recommended. Trying to resolve relative to current directory.")

        self.locations_file = file_path
        self._file_lock = threading.Lock()
        self.locations = self._load_locations()

        # Create services
        self.save_srv = self.create_service(SaveLocation, 'save_location', self._save_location_callback)
        self.get_srv = self.create_service(GetLocation, 'get_location', self._get_location_callback)
        self.delete_srv = self.create_service(DeleteLocation, 'delete_location', self._delete_location_callback)
        self.list_srv = self.create_service(ListLocations, 'list_locations', self._list_locations_callback)

    def _load_locations(self):
        with self._file_lock:
            try:
                if os.path.exists(self.locations_file):
                    with open(self.locations_file, 'r') as f:
                        self.get_logger().info(f"Loading locations from {self.locations_file}")
                        return json.load(f)
                else:
                    self.get_logger().info(f"Locations file not found at {self.locations_file}. Starting with an empty list.")
                    return {}
            except (json.JSONDecodeError, IOError) as e:
                self.get_logger().error(f"Failed to load locations file: {e}. Starting fresh.")
                return {}

    def _save_locations(self):
        # Using atomic write (write to temp file then rename) to prevent data corruption
        temp_file = self.locations_file + '.tmp'
        with self._file_lock:
            try:
                with open(temp_file, 'w') as f:
                    json.dump(self.locations, f, indent=4)
                os.rename(temp_file, self.locations_file)
                return True
            except IOError as e:
                self.get_logger().error(f"Failed to save locations file: {e}")
                if os.path.exists(temp_file):
                    os.remove(temp_file) # Clean up temp file
                return False

    def _save_location_callback(self, request, response):
        location_name = request.name.strip()
        if not location_name:
            response.success = False
            response.message = "Location name cannot be empty."
            return response

        self.get_logger().info(f"Received request to save location: '{location_name}'")
        
        # Policy: Overwrite if name already exists
        is_update = location_name in self.locations
        
        self.locations[location_name] = {
            "pose": pose_stamped_to_dict(request.pose),
            "map_id": "default_map" # Placeholder for map versioning
        }

        if self._save_locations():
            response.success = True
            if is_update:
                response.message = f"Successfully updated location '{location_name}'."
                self.get_logger().info(f"Updated location: '{location_name}'")
            else:
                response.message = f"Successfully saved new location '{location_name}'."
                self.get_logger().info(f"Saved new location: '{location_name}'")
        else:
            response.success = False
            response.message = "Failed to write locations to file."
            # Attempt to roll back the change in memory
            # Note: This is a simple rollback. A more robust system might reload from the file.
            self._load_locations()

        return response

    def _get_location_callback(self, request, response):
        location_name = request.name.strip()
        self.get_logger().info(f"Received request to get location: '{location_name}'")
        
        location_data = self.locations.get(location_name)

        if location_data:
            try:
                response.pose = dict_to_pose_stamped(location_data['pose'])
                response.success = True
            except (KeyError, TypeError) as e:
                response.success = False
                self.get_logger().error(f"Failed to parse pose for '{location_name}': {e}")
        else:
            response.success = False
            self.get_logger().warn(f"Location '{location_name}' not found.")
            
        return response

    def _delete_location_callback(self, request, response):
        location_name = request.name.strip()
        self.get_logger().info(f"Received request to delete location: '{location_name}'")

        if location_name in self.locations:
            del self.locations[location_name]
            if self._save_locations():
                response.success = True
                response.message = f"Successfully deleted location '{location_name}'."
                self.get_logger().info(f"Deleted location: '{location_name}'")
            else:
                response.success = False
                response.message = "Failed to write locations to file after deletion."
                self._load_locations() # Rollback
        else:
            response.success = False
            response.message = f"Location '{location_name}' not found."
            self.get_logger().warn(f"Attempted to delete non-existent location: '{location_name}'")

        return response

    def _list_locations_callback(self, request, response):
        self.get_logger().info("Received request to list all locations.")
        names = []
        poses = []
        for name, data in self.locations.items():
            try:
                names.append(name)
                poses.append(dict_to_pose_stamped(data['pose']))
            except (KeyError, TypeError) as e:
                self.get_logger().error(f"Skipping location '{name}' in list due to parsing error: {e}")

        response.names = names
        response.poses = poses
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LocationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
