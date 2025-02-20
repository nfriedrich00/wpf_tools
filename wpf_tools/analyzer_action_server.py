import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from wpf_msgs.action import AnalyzeLogs

from wpf_tools.analyzer_node import AnalyzerNode

class AnalyzerActionServer(Node):

    def __init__(self):
        super().__init__('analyzer_action_server')
        self._action_server = ActionServer(
            self,
            AnalyzeLogs,
            'analyze_logs',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('AnalyzerActionServer is ready.')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        goal = goal_handle.request
        print(goal.overwrite_results)

        analyzer = AnalyzerNode()
        if goal.logs_directory:
            analyzer.logs_dir = goal.logs_directory

        analyzer.overwrite_results = goal.overwrite_results
        if goal.start_time > 0 or goal.end_time > 0:
            analyzer.start_time = goal.start_time
            analyzer.end_time = goal.end_time

        try:
            analyzer.analyze_data()
            goal_handle.succeed()
            return AnalyzeLogs.Result(success=True)
        except Exception as e:
            self.get_logger().error(f'Analysis failed: {str(e)}')
            goal_handle.abort()
            return AnalyzeLogs.Result(success=False)

def main(args=None):
    rclpy.init(args=args)
    action_server = AnalyzerActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
