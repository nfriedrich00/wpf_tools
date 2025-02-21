""" Analyze data from claudi simulation to give out and save additional
information like average speed, traveled distance and error.
"""
import os
import glob
import rclpy
from rclpy.node import Node
from wpf_tools.log_analyzer import LogAnalyzer


class AnalyzerNode(Node):
    """ Analyzer Node, which analyzes the data logged during any
    claudi simulation. Per default, it analyzes the data of the last
    simulation and generates and logs some additional data,
    like average speed, average/max error and traveled distance.
    """

    def __init__(self):
        super().__init__('analyzer_node')

        default_logs_parent_dir = os.path.expanduser("~") + "/Documents/wpf/logs"

        self.declare_parameter('logs_directory', '')
        self.logs_dir = self.get_parameter('logs_directory').value
        if self.logs_dir:
            # True if parameter is set
            self.get_logger().info(f'Logs directory provided: {self.logs_dir}')
        else:
            # Parameter not set, using default value
            #default_logs_dir = ... # todo: move here
            self.get_logger().info(f'No logs directory provided. Looking for logs in default directory {default_logs_parent_dir}.')
            if not os.path.exists(default_logs_parent_dir):
                self.get_logger().error(f'Default logs directory {default_logs_parent_dir} does not exist. Aborting.')
                raise FileNotFoundError
            list_of_log_dirs = [f for f in glob.glob(default_logs_parent_dir + '/*') if os.path.basename(f).isdigit()]
            if not list_of_log_dirs:
                self.get_logger().error(f'No logs found in default directory {default_logs_parent_dir}. Aborting.')
                raise FileNotFoundError
            path_of_newest_log = max(
                list_of_log_dirs,
                key=lambda x: int(os.path.basename(x)))
            self.logs_dir = path_of_newest_log
            self.get_logger().info(f'Using logs from latest session: {self.logs_dir}')


        self.declare_parameter('overwrite_results', True)
        self.overwrite_results = self.get_parameter('overwrite_results').value
        """
        Parameter to decide if an existing results file should be overwritten. Otherwise, a new file with a number suffix is created.
        """

        self.declare_parameter('start_time', 0.0)
        self.start_time = self.get_parameter('start_time').value
        """
        Parameter to limit the analysis to a specific time interval.
        The analysis will only consider data with a timestamp greater or equal to this value.
        Set to 0.0 to disable.
        """

        self.declare_parameter('end_time', 0.0)
        self.end_time = self.get_parameter('end_time').value
        """
        Parameter to limit the analysis to a specific time interval.
        The analysis will only consider data with a timestamp smaller or equal to this value.
        Set to 0.0 to disable.
        """

        if self.end_time < self.start_time:
            self.get_logger().error('End time is smaller than start time. Aborting.')
            raise ValueError
        
        self.create_timer(0.1, self.run_analysis)
        # Use this timer instead of a function call.
        # This way, the node will be destroyed after the analysis is done without errors.

    def run_analysis(self):
        log_analyzer = LogAnalyzer(self.logs_dir, self.overwrite_results, self.start_time, self.end_time)
        if log_analyzer.analyze_data():
            self.get_logger().info("Analysis successful.")
        else:
            self.get_logger().error("Analysis failed.")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """ Analyze data from claudi simulation to give out and save
    information like average speed, traveled distance and error.
    """
    rclpy.init(args=args)
    analyzer_node = AnalyzerNode()
    rclpy.spin_once(analyzer_node) # spine_once: self destruct after analysis
    #analyzer_node.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
