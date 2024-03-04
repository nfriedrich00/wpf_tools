import rclpy
import yaml
from rclpy.node import Node
from math import ceil, cos, sqrt
from copy import deepcopy
from robot_localization.srv import ToLL
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

class WaypointGenerator(Node):
    mysterious_offset_in_x_direction_in_m = -0.219 # this is - y offset from localization?
    mysterious_offset_in_y_direction_in_m = -0.127

    def __init__(self):
        super().__init__('waypoint_generator_node')

        self.declare_parameter('pattern', 'line')
        self.pattern = self.get_parameter('pattern').value
        self.declare_parameter('waypoint_separation_in_m', 0.2)
        self.declare_parameter('intended_length_in_m', 5.0)
        self.declare_parameter('use_length_in_propagation_direction', False)
        self.declare_parameter('cosine_amplitude_in_m', 1.0)
        self.declare_parameter('cosine_wavelength_in_m', 1.0)

        self.cli = self.create_client(ToLL, 'toLL')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ToLL.Request()
        
        self.PI = 3.141592654
        self.test = 1
        self.start_point = Point()
        self.waypoints = [self.start_point]

    def generate_line(self, intended_length_in_m = None, waypoint_separation_in_m = None):
        if intended_length_in_m is None:
            intended_length_in_m = self.get_parameter('intended_length_in_m').value
        else: intended_length_in_m = intended_length_in_m
        if waypoint_separation_in_m is None:
            waypoint_separation_in_m = self.get_parameter('waypoint_separation_in_m').value
        else: waypoint_separation_in_m = waypoint_separation_in_m


        for point_number in range(1, ceil(intended_length_in_m/waypoint_separation_in_m) + 1):
            point = Point()
            point.x = self.start_point.x + point_number*waypoint_separation_in_m
            point.y = self.start_point.y
            self.waypoints.append(deepcopy(point))

        #self.generate_yaml(waypoints=self.waypoints)

    def generate_cosine(self, intended_length_in_m = None, waypoint_separation_in_m = None, cosine_amplitude_in_m = None, cosine_wavelength_in_m = None, use_length_in_propagation_direction = None):
        def get_parameter(param_name, default_value=None):
            if default_value is None:
                return self.get_parameter(param_name).value
            else:
                return default_value
            
        intended_length_in_m = get_parameter('intended_length_in_m', intended_length_in_m)
        use_length_in_propagation_direction = get_parameter('use_length_in_propagation_direction', use_length_in_propagation_direction)
        waypoint_separation_in_m = get_parameter('waypoint_separation_in_m', waypoint_separation_in_m)
        cosine_amplitude_in_m = get_parameter('cosine_amplitude_in_m', cosine_amplitude_in_m)
        cosine_wavelength_in_m = get_parameter('cosine_wavelength_in_m', cosine_wavelength_in_m)

        def get_y_value(x):
            y = cosine_amplitude_in_m*cos((x * 2*self.PI)/(cosine_wavelength_in_m)) - cosine_amplitude_in_m
            return y

        def get_x_separation():
            x_separation = waypoint_separation_in_m
            for iteration in range(0, 4):
                y_separation = abs(self.waypoints[-1].y - get_y_value(self.waypoints[-1].x + x_separation))
                distance = sqrt((x_separation) ** 2 + (y_separation) ** 2)
                x_correction = waypoint_separation_in_m / distance
                x_separation *= x_correction
            return x_separation

        def is_calulation_finished():
            if use_length_in_propagation_direction:
                return self.waypoints[-1].x > intended_length_in_m
            else:
                return len(self.waypoints) > (intended_length_in_m / waypoint_separation_in_m)


        #number_of_points = intended_length_in_m / waypoint_separation_in_m
            
        while not is_calulation_finished():
            next_point = Point()
            next_point.x = self.waypoints[-1].x + get_x_separation()
            next_point.y = get_y_value(next_point.x)
            self.waypoints.append(next_point)


        #self.generate_yaml(waypoints=self.waypoints)

    def convert_waypoints_to_LL(self, waypoints):
        geopoint = GeoPoint()
        gps_waypoints = []
        for map_waypoint in waypoints:
            #convert to LL
            self.req.map_point:Point = map_waypoint
            #mysterious offset, why are x and y directions and sign switched in comparison to localization?
            self.req.map_point.x += self.mysterious_offset_in_x_direction_in_m
            self.req.map_point.y += self.mysterious_offset_in_y_direction_in_m
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            geopoint = self.future.result().ll_point
            gps_waypoints.append(deepcopy(geopoint))
        return gps_waypoints



    def generate_gps_yaml(self, waypoints):
        waypoints_yaml = [{'latitude': geopoint.latitude, 'longitude': geopoint.longitude, 'yaw': 0.0} for geopoint in waypoints]
        data = {'waypoints': waypoints_yaml}
        
        with open('waypoints_out.yaml', 'w') as file:
            yaml.dump(data, file, default_flow_style=False)

    #only test
    def plot_waypoints(self, waypoints):
        x = []
        y = []
        for point in waypoints:
            x.append(point.x)
            y.append(point.y)

        #tests
        import matplotlib.pyplot as plt

        print('Values of x: ', x)
        print('Values of y: ', y)
        plt.plot(x, y)
        plt.title("Waypoints")
        plt.xlabel("Values of x")
        plt.ylabel("Values of y")
        plt.show()





        


def main(args=None):
    rclpy.init(args=args)

    waypoint_generator = WaypointGenerator()
    if waypoint_generator.pattern.lower() == 'cosine':
        waypoint_generator.generate_cosine()
    elif waypoint_generator.pattern.lower() == 'square wave':
        raise NotImplementedError
    else:
        waypoint_generator.generate_line()

    waypoint_generator.waypoints = waypoint_generator.convert_waypoints_to_LL(waypoints = waypoint_generator.waypoints)
    waypoint_generator.generate_gps_yaml(waypoints = waypoint_generator.waypoints)
    #waypoint_generator.plot_waypoints(waypoints = waypoint_generator.waypoints)

if __name__ == '__main__':
    main()