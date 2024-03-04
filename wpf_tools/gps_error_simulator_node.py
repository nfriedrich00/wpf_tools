import os
import datetime
import random
import threading
from math import sin, cos, pi
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class GPSErrorSimulator(Node):
    message = NavSatFix()
    # Add this to the lat/long in degree to add 1cm in Freiberg (50.91836/13.341)
    lat_1_cm = 8.9928e-6
    long_1_cm = 1.42653e-5

    def __init__(self):
        super().__init__('gps_error_simulator')
        time_now = datetime.datetime.now()
        time_now_string = f"{time_now.year}{time_now.month:02d}{time_now.day:02d}{time_now.hour:02d}{time_now.minute:02d}{time_now.second:02d}"
        self.declare_parameter('session_start_time_string', time_now_string).value
        self.session_start_time_string = self.get_parameter('session_start_time_string').value
        self.declare_parameter('input_topic', '/emlid/fix')
        self.declare_parameter('output_topic', '/emlid/modified')
        input_topic_value = self.get_parameter('input_topic').value
        output_topic_value = self.get_parameter('output_topic').value
        if type(input_topic_value) != str or type(output_topic_value) != str:
            self.get_logger().info(type(input_topic_value))
            raise ValueError
        input_topic:String = input_topic_value
        output_topic:String = output_topic_value

        self.declare_parameter('enable_mysterious_offset_correction', True)
        self.mysterious_offset_correction_enabled = self.get_parameter('enable_mysterious_offset_correction').value
        self.declare_parameter('enable_logs', True)
        self.logs_enabled = self.get_parameter('enable_logs').value

        self.declare_parameter('enable_noise', False)
        self.declare_parameter('enable_offset', False)
        self.declare_parameter('enable_spikes', False)
        self.declare_parameter('enable_delay', False)

        self.declare_parameter('enable_covariance_set', False)      # set covariance, additional increase is possible, overwritten by truthful covariance
        self.declare_parameter('enable_covariance_increase', False) # increase covariance (0 per default if not set enabled), overwritten by truthful covariance
        self.declare_parameter('enable_truthful_covariance', False) # only effective if noise, spikes or offset enabled. Sets covariance corresponding to error. Overwrites set covariance

        # Noise
        distance_multiplicator = 0.1
        confidence_multiplicator = 0.33
        self.declare_parameter('noise_sigma', distance_multiplicator * confidence_multiplicator)
        self.declare_parameter('restrain_noise_to_x_direction', False)
        self.declare_parameter('restrain_noise_to_y_direction', False)

        # Offset
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)

        # Delay
        self.declare_parameter('delay', 0.0)

        # Spikes
        self.declare_parameter('spikes_probability', 0.0)
        self.declare_parameter('spikes_mean', 0.0)
        self.declare_parameter('spikes_std', 0.0)
        self.declare_parameter('spikes_direction', None) # leave empty for random direction

        # Covariance
        self.declare_parameter('covariance_increase_x', 0.0)
        self.declare_parameter('covariance_increase_y', 0.0)
        self.declare_parameter('covariance_set_x', 0.0)
        self.declare_parameter('covariance_set_y', 0.0)


        # Modifier
        self.x_modifier_in_m = 0.0
        self.y_modifier_in_m = 0.0

        self.covariance_x = 0.0
        self.covariance_y = 0.0

        self.variance_x = float()
        self.variance_y = float()


        # Runtime error when logging set to true after init. todo create log folder when param set to true at runtime?
        if self.logs_enabled:
            self.start_logging()

        # Always create message queue, evene when delay disabled. Otherwise delay can not be actived at runtime. todo make it work?
        self.create_message_queue()

        #Subscriber NavSatFix
        self.subscription = self.create_subscription(
            NavSatFix,
            input_topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(f"subscription created for topic {input_topic}")
        self.subscription


        #Publisher
        self.publisher_ = self.create_publisher(NavSatFix, output_topic, 10)

        self.get_logger().info("init finished")


    def start_logging(self):
        session_identifier = self.session_start_time_string
        self.logs_path = os.path.expanduser("~") + "/Documents" + "/wpf/logs/gps_error_simulator" + f"/{session_identifier}"

        if not os.path.exists(self.logs_path):
            os.makedirs(self.logs_path)
            self.get_logger().info(f"Start logging, files can be found in {self.logs_path}")
        else:
            self.get_logger().info(f"Logs folder {self.logs_path} already exists. This should not be and can result in corrupted logs or runtime errors.")

    def create_message_queue(self):
        self.message_queue = []

    def listener_callback(self, msg):
        self.x_modifier_in_m = 0.0
        self.y_modifier_in_m = 0.0

        self.covariance_x = 0.0
        self.covariance_y = 0.0
        if self.mysterious_offset_correction_enabled:
            msg = self.correct_mysterious_offset(msg) # This is now the original message. There never was some unexplainable error

        if self.logs_enabled:
            self.log_message_received(original_message = msg)
        
        self.process_message(original_message = msg)
        # process message includes: calculation of all selected error types, application and publishing/queuing

    
    def correct_mysterious_offset(self, msg): # mysterious offset when using gps localization why?
        offset_in_x_direction_in_m = 0.127
        offset_in_y_direction_in_m = 0.219
        msg.latitude += self.lat_1_cm * offset_in_y_direction_in_m
        msg.longitude += self.long_1_cm * offset_in_x_direction_in_m
        return msg

    def process_message(self, original_message):
        noise_enabled = self.get_parameter('enable_noise').value
        if noise_enabled:
            noise = self.calculate_noise_in_m()
            noise_x_direction = noise[0]
            noise_y_direction = noise[1]
            self.x_modifier_in_m += noise_x_direction
            self.y_modifier_in_m += noise_y_direction
            if self.logs_enabled:
                self.log_noise(timestamp_message=original_message.header.stamp, error_x=noise_x_direction, error_y=noise_y_direction)

        offset_enabled = self.get_parameter('enable_offset').value
        if offset_enabled:
            offset_x_direction = self.get_parameter('offset_x').value
            offset_y_direction = self.get_parameter('offset_y').value
            self.x_modifier_in_m += offset_x_direction
            self.y_modifier_in_m += offset_y_direction
            if self.logs_enabled:
                self.log_offset(timestamp_message=original_message.header.stamp, offset_x=offset_x_direction, offset_y=offset_y_direction)

        spikes_enabled = self.get_parameter('enable_spikes').value
        if spikes_enabled:
            spike = self.calculate_spike_in_m()
            spike_x_direction = spike[0]
            spike_y_direction = spike[1]
            self.x_modifier_in_m += spike_x_direction
            self.y_modifier_in_m += spike_y_direction
            if self.logs_enabled:
                self.log_spike(timestamp_message = original_message.header.stamp, spike_x = spike_x_direction, spike_y = spike_y_direction) 

        self.calculate_covariance()
        if self.logs_enabled:
            self.log_covariance(timestamp_message = original_message.header.stamp)

        delay_enabled = self.get_parameter('enable_delay').value
        if delay_enabled:
            self.message_queue.append(original_message)
            timer = threading.Timer(self.get_parameter('delay').value, self.send_message, args=(self.message_queue.pop(0),))
            timer.start()
        else:
            self.send_message(unmodified_message = original_message)


    def calculate_noise_in_m(self):
        gauss_sigma = self.get_parameter('noise_sigma').value
        # gauss sigma is set in m. 99.7% of errors less than 10cm => sigma = 1/3 * 0.1 is the same as 95% less than 6.6cm => sigma = 1/2 * 0.066
        # error is generated using this sigma, result in m, later converted to lat/long using reference in Freiberg, Germany (50.91836/13.3410) and not a real wgs - map transformation
        noise_distance = random.gauss(0.0, gauss_sigma)
        if self.get_parameter('restrain_noise_to_x_direction').value:
            noise_direction = 0.0
        elif self.get_parameter('restrain_noise_to_y_direction').value:
            noise_direction = pi/2
        else:
            noise_direction = random.uniform(0, 2 * pi)
        noise_x_in_m = noise_distance * cos(noise_direction)
        noise_y_in_m = noise_distance * sin(noise_direction)
        gauss_sigma_x = gauss_sigma * cos(noise_direction)
        gauss_sigma_y = gauss_sigma * sin(noise_direction)
        self.variance_x = gauss_sigma_x ** 2
        self.variance_y = gauss_sigma_y ** 2
        return [noise_x_in_m, noise_y_in_m]
    
    def calculate_spike_in_m(self):
        if random.random() < self.get_parameter('spikes_probability').value:
            spike_distance = random.gauss(self.get_parameter('spikes_mean').value, self.get_parameter('spikes_std').value)
            spike_direction = self.get_parameter('spikes_direction').value if self.get_parameter('spikes_direction').value is not None else random.uniform(0, 2 * pi)
            spike_x_in_m = spike_distance * cos(spike_direction)
            spike_y_in_m = spike_distance * sin(spike_direction)
        else:
            spike_x_in_m = 0.0
            spike_y_in_m = 0.0
        return [spike_x_in_m, spike_y_in_m]
    
    def calculate_covariance(self):
        set_covariance_enabled = self.get_parameter('enable_covariance_set').value
        add_to_covariance_enabled = self.get_parameter('enable_covariance_increase').value
        truthful_covariance_enabled = self.get_parameter('enable_truthful_covariance').value
        if not (truthful_covariance_enabled or set_covariance_enabled or add_to_covariance_enabled):
            return
        
        if truthful_covariance_enabled:
            self.covariance_x = self.variance_x
            self.covariance_y = self.variance_y
            return

        if set_covariance_enabled:
            self.covariance_x = self.get_parameter('covariance_set_x').value
            self.covariance_y = self.get_parameter('covariance_set_y').value
        
        if add_to_covariance_enabled:
            self.covariance_x += self.get_parameter('covariance_increase_x').value
            self.covariance_y += self.get_parameter('covariance_increase_y').value


    
    def send_message(self, unmodified_message:NavSatFix):
        message_to_send = unmodified_message
        message_to_send.latitude += self.y_modifier_in_m * self.lat_1_cm
        message_to_send.longitude += self.x_modifier_in_m * self.long_1_cm
        message_to_send.position_covariance_type = 2 # = COVARIANCE_TYPE_DIAGONAL_KNOWN
        # x direction is east per default, y is north
        # covariance matrix/list[9]: (ENU - east-north-up = x-y-z
        # long_in_m (east)     0.0          0.0
        #       0.0      lat_in_m (north)   0.0
        #       0.0            0.0      alt_in_m (up)
        message_to_send.position_covariance[0] = self.covariance_x
        message_to_send.position_covariance[4] = self.covariance_y
        self.publisher_.publish(message_to_send)
        if self.logs_enabled:
            self.log_message_sent(modified_message=message_to_send)


    def log_message_received(self, original_message):
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')
        #message_time_float = float(f'{original_message.header.stamp.sec}.{original_message.header.stamp.nanosec}')


        yaml_data = {time_now_float: {'time': float(f'{original_message.header.stamp.sec}.{original_message.header.stamp.nanosec}'),
                                    'frame_id': str(original_message.header.frame_id),
                                    'latitude': original_message.latitude,
                                    'longitude': original_message.longitude,
                                    'altitude': original_message.altitude,
                                    'position_covariance': [float(original_message.position_covariance[i]) for i in range(0, len(original_message.position_covariance))],
                                    'position_covariance_type': original_message.position_covariance_type
                                    }}
        
        with open((self.logs_path + '/received.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)


    def log_message_sent(self, modified_message):
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')

        yaml_data = {time_now_float: {'time': float(f'{modified_message.header.stamp.sec}.{modified_message.header.stamp.nanosec}'),
                                    'frame_id': str(modified_message.header.frame_id),
                                    'latitude': modified_message.latitude,
                                    'longitude': modified_message.longitude,
                                    'altitude': modified_message.altitude,
                                    'position_covariance': [float(modified_message.position_covariance[i]) for i in range(0, len(modified_message.position_covariance))],
                                    'position_covariance_type': modified_message.position_covariance_type
                                    }}

        with open((self.logs_path + '/sent.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)


    def log_noise(self, timestamp_message, error_x, error_y):
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')
        time_message_float = float(f'{timestamp_message.sec}.{timestamp_message.nanosec}')

        yaml_data = {time_now_float: {'time': time_message_float,
                                    'x_modifier': error_x,
                                    'y_modifier': error_y
                                    }}
        
        with open((self.logs_path + '/noise.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)


    def log_offset(self, timestamp_message, offset_x, offset_y):
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')
        time_message_float = float(f'{timestamp_message.sec}.{timestamp_message.nanosec}')

        yaml_data = {time_now_float: {'time': time_message_float,
                                    'x_modifier': offset_x,
                                    'y_modifier': offset_y
                                    }}

        with open((self.logs_path + '/offset.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)


    def log_spike(self, timestamp_message, spike_x, spike_y):
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')
        time_message_float = float(f'{timestamp_message.sec}.{timestamp_message.nanosec}')

        yaml_data = {time_now_float: {'time': time_message_float,
                                    'x_modifier': spike_x,
                                    'y_modifier': spike_y
                                    }}
        
        with open((self.logs_path + '/spikes.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)


    def log_covariance(self, timestamp_message):
        time_now = self.get_clock().now().to_msg()
        time_now_float = float(f'{time_now.sec}.{time_now.nanosec}')
        time_message_float = float(f'{timestamp_message.sec}.{timestamp_message.nanosec}')

        yaml_data = {time_now_float: {'time': time_message_float,
                                    'x_covariance': self.covariance_x,
                                    'y_covariance': self.covariance_y,
                                    'is_truthful': self.get_parameter('enable_truthful_covariance').value
                                    }}
        
        with open((self.logs_path + '/covariance.yaml'), 'a') as logfile:
            yaml.dump(yaml_data, logfile, default_flow_style=False)


def main(args=None):
    rclpy.init(args=args)
    gps_error_simulator = GPSErrorSimulator()
    rclpy.spin(gps_error_simulator)
    gps_error_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
