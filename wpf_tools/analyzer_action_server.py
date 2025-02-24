import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from wpf_msgs.action import AnalyzeLogs

from wpf_tools.analyzer_node import LogAnalyzer

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
        self.get_logger().info('Analyzer action server is ready.')

    def goal_callback(self, goal_request):
        if not goal_request.logs_directory:
            self.get_logger().error('Rejecting goal request: No logs directory provided. ')
            return GoalResponse.REJECT
        if goal_request.start_time > 0 and goal_request.end_time > 0 and goal_request.start_time > goal_request.end_time:
            self.get_logger().error('Rejecting goal request: Start time is greater than end time.')
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        #goal_handle.canceled() # not tested
        return CancelResponse.ACCEPT
    
    def give_feedback(self, goal_handle, feedback: str):
        self.get_logger().info(feedback)
        feedback_msg = AnalyzeLogs.Feedback()
        feedback_msg.feedback = feedback
        goal_handle.publish_feedback(feedback_msg)

    def execute_callback(self, goal_handle):
        self.give_feedback(goal_handle, 'Received goal request.')
        goal = goal_handle.request
        analyzer = LogAnalyzer(goal.logs_directory, goal.overwrite_results,
                               goal.start_time, goal.end_time,
                               goal.start_position, goal.end_position)

        self.give_feedback(goal_handle, 'Loading data...')

        if analyzer.load_data():
            self.give_feedback(goal_handle, 'Data loaded successfully.')
        else:
            self.give_feedback(goal_handle, 'Failed to load data.')
            goal_handle.abort()
            return AnalyzeLogs.Result(success=False)

        self.give_feedback(goal_handle, 'Restricting data to relevant interval...')

        if analyzer.limit_data():
            self.give_feedback(goal_handle, 'Success')
        else:
            self.give_feedback(goal_handle, 'Failed to limit data.')
            goal_handle.abort()
            return AnalyzeLogs.Result(success=False)

        self.give_feedback(goal_handle, 'Calculating errors from logs...')

        if analyzer.get_errors():
            self.give_feedback(goal_handle, 'Succesfully calculated errors from logs.')
        else:
            self.give_feedback(goal_handle, 'Failed to calculate errors from logs.')
            return AnalyzeLogs.Result(success=False)
        
        self.give_feedback(goal_handle, 'Saving results...')

        if analyzer.log_results(analyzer.results):
            self.give_feedback(goal_handle, f'Results saved to {analyzer.results_path}')
            goal_handle.succeed()
            return AnalyzeLogs.Result(success=True)
        else:
            self.give_feedback(goal_handle, f'Failed to save results to {analyzer.results_path}')
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
