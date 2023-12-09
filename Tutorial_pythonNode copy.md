<!--
author:  André Dietrich; Sebastian Zug

mode:   Presentation

comment: Interactive LiaScript workshop at Federal University of Amazonas

-->

# Python ROS2 Nodes

**Packages** - Packages encapsulate individual algorithms and realise their dependencies. Ultimately, this ensures the reusability of an implementation.

**Nodes** - A node is a participant in the ROS graph. ROS nodes use a ROS client library to communicate with other nodes. Nodes are structured in packages.

```bash
> ros2 run <package> <node>
> ros2 run turtlesim turtlesim_node
> ros2 run turtlesim turtle_teleop_key
```

## Python ROS2 Nodes

```python   publisher.py
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

```python     subscriber.py
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Package configuration

According to the documentation of [ROS2](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html), the following steps are necessary to build our own package:

0. Create a new workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

1. Create a new package

```
ros2 pkg create --build-type ament_python py_manaus_demo
```

2. Add your code as `publisher.py` and `subscriber.py` to the `py_manaus_demo` folder.

3. Replace the empty `console_scripts` line in `setup.py` file by

```python
entry_points={
        'console_scripts': [
                'talker = py_manaus_demo.publisher:main',
                'listener = py_manaus_demo.subscriber:main',
        ],
},
```

4. Add the dependencies to the `package.xml` file

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

5. Build the package. Switch to the workspace folder and run

```bash
colcon build --packages-select py_manaus_demo
```

6. Source the workspace

> Pay attention this step depends on the shell you are using. For bash, run `setup.bash`, for zsh, run `setup.zsh` ...

```bash
source ~/ros2_ws/install/setup.bash
```