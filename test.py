import inspect
import std_msgs.msg as ros_msg  # importa il tuo file
from tactigon_shapes.modules.ros2.extension import Ros2Process
from tactigon_shapes.modules.ros2.models import Ros2ShapeConfig, Ros2Publisher, Ros2Subscription

i = 0

def echo(msg):
    global i
    i += 1
    print(i, msg)

cfg = Ros2ShapeConfig(
    "test_shape",
    [
        Ros2Publisher("test2", ros_msg.Int64)
    ],
    [
        Ros2Subscription("/recv/data", "test_func", "payload_ref", ros_msg.String)
    ]
)


def main():
    with Ros2Process(cfg, echo) as ros:
        while True:
            msg = input("Messaggio da inviare: ")
            ros.publish("test2", ros_msg.Int64, int(msg))


if __name__ == "__main__":
    main()