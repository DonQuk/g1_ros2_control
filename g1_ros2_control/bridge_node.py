import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 

from unitree_sdk2py.core.channel import ChannelFactory

try:
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
except ImportError as e:
    print(f"SDK Error: {e}")
    LocoClient = None
    G1ArmActionClient = None

class G1Ros2Bridge(Node):
    def __init__(self):
        super().__init__('g1_bridge')
        
        self.declare_parameter('network_interface', 'enx00e02d682cc6')
        self.network_interface = self.get_parameter('network_interface').get_parameter_value().string_value
        
        self.get_logger().info(f"Initializing Bridge on: {self.network_interface}")

        if LocoClient is None or G1ArmActionClient is None:
            self.get_logger().error("Critical: SDK modules missing.")
            return

        try:
            self.channel_factory = ChannelFactory()
            self.channel_factory.Init(0, self.network_interface)
            
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(10.0)
            self.loco_client.Init()
            
            self.arm_client = G1ArmActionClient()
            self.arm_client.SetTimeout(10.0)
            self.arm_client.Init()
            
            self.get_logger().info("✅ G1 SDK Initialized!")

        except Exception as e:
            self.get_logger().error(f"SDK Init Failed: {e}")
            raise e

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            
        self.action_sub = self.create_subscription(
            Int32, '/action', self.action_callback, 10)
        
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        
        self.timer = self.create_timer(0.01, self.control_loop)

    def cmd_vel_callback(self, msg):
        self.vx = max(min(msg.linear.x, 0.5), -0.5)
        self.vy = -max(min(msg.linear.y, 0.3), -0.3)
        self.vyaw = -max(min(msg.angular.z, 0.5), -0.5)

    def action_callback(self, msg):
        action_id = msg.data
        self.get_logger().info(f"Received Action ID: {action_id}")
        
        try:
            ret = self.arm_client.ExecuteAction(action_id)
            self.get_logger().info(f"Action Executed. Return Code: {ret}")
        except Exception as e:
            self.get_logger().error(f"Action Execution Failed: {e}")

    def control_loop(self):
        try:
            self.loco_client.Move(self.vx, self.vy, self.vyaw)
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    try:
        node = G1Ros2Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node error: {e}")
    finally:
        try:
            if 'node' in locals():
                node.loco_client.Move(0.0, 0.0, 0.0)
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()