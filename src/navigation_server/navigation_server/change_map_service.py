import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import FinishTrajectory, DeleteTrajectory, LoadStateFromFile, StartTrajectory
from beefast_interfaces.srv import ChangeMap


class NavigationServer(Node):

    def __init__(self):
        super().__init__('navigation_server')
        self.get_logger().info('Starting Navigation Server...')

        # Create service
        self.change_map_service = self.create_service(ChangeMap, '/change_map', self.change_map_callback)

        # Initialize clients
        self.finish_client = None
        self.delete_trajectory_client = None
        self.load_pbstream_client = None
        self.init_pose_client = None

    def change_map_callback(self, request, response):
        self.get_logger().warn('Starting map change process...')

        # 1) Finish trajectory
        if not self.finish_client:
            self.finish_client = self.create_client(FinishTrajectory, '/finish_trajectory')
        if not self.finish_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'finish_trajectory' not available.")
            response.status = "failure"
            return response

        finish_request = FinishTrajectory.Request()
        finish_request.trajectory_id = 0
        self.finish_client.call_async(finish_request)

        # 2) Delete trajectory
        if not self.delete_trajectory_client:
            self.delete_trajectory_client = self.create_client(DeleteTrajectory, '/delete_trajectory')
        if not self.delete_trajectory_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'delete_trajectory' not available.")
            response.status = "failure"
            return response

        delete_request = DeleteTrajectory.Request()
        delete_request.trajectory_id = 0
        self.delete_trajectory_client.call_async(delete_request)

        # 3) Load new trajectory
        if not self.load_pbstream_client:
            self.load_pbstream_client = self.create_client(LoadStateFromFile, '/load_state_from_file')
        if not self.load_pbstream_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'load_state_from_file' not available.")
            response.status = "failure"
            return response

        load_request = LoadStateFromFile.Request()
        load_request.file_path = request.file_path
        load_request.load_frozen_state = True
        self.load_pbstream_client.call_async(load_request)

        # 4) Initialize pose
        if not self.init_pose_client:
            self.init_pose_client = self.create_client(StartTrajectory, '/start_trajectory')
        if not self.init_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'start_trajectory' not available.")
            response.status = "failure"
            return response

        init_pose_request = StartTrajectory.Request()
        init_pose_request.configuration_directory = "/opt/beefast/config/"
        init_pose_request.configuration_basename = "cartographer_localization.lua"
        init_pose_request.initial_pose = request.initial_pose
        init_pose_request.relative_to_trajectory_id = 0
        self.init_pose_client.call_async(init_pose_request)

        # Finalize
        response.status = "success"
        self.get_logger().info("Map change completed successfully.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()