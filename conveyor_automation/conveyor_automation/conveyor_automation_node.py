import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from conveyorbelt_msgs.srv import ConveyorBeltControl

class ConveyorAutomationNode(Node):
    def __init__(self):
        super().__init__('conveyor_automation_node')
        
        # Subscribe to sensor topics
        self.create_subscription(LaserScan, '/conveyor_sensor_ir1/ir_sensor1', self.ir_sensor1_callback, 10)
        self.create_subscription(LaserScan, '/conveyor_sensor_ir2/ir2_sensor', self.ir_sensor2_callback, 10)
        
        # Create client for the conveyor power service
        self.conveyor_power_client = self.create_client(ConveyorBeltControl, '/CONVEYORPOWER')
        
        self.get_logger().info('Conveyor Automation Node initialized')

    def ir_sensor1_callback(self, msg: LaserScan):
        if self.is_object_detected(msg):
            self.call_conveyor_power(20.0)  # Changed to float

    def ir_sensor2_callback(self, msg: LaserScan):
        if self.is_object_detected(msg):
            self.call_conveyor_power(0.0)  # Changed to float

    def is_object_detected(self, msg: LaserScan):
        # Implement logic to interpret LaserScan data
        # This is a simple example; adjust based on your sensor's specifics
        threshold_distance = 0.5  # Adjust this value based on your setup
        return any(range < threshold_distance for range in msg.ranges if range > msg.range_min)

    def call_conveyor_power(self, power: float):
        request = ConveyorBeltControl.Request()
        request.power = power
        
        future = self.conveyor_power_client.call_async(request)
        future.add_done_callback(self.conveyor_power_callback)

    def conveyor_power_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorAutomationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
