from std_msgs.msg import String, Int64

from tactigon_shapes.modules.ros2.extension import Ros2Interface
from tactigon_shapes.modules.ros2.models import Ros2Config, Ros2Publisher, Ros2Subscription

i = 0

def echo(msg):
    global i
    i += 1
    print(i, msg)

cfg = Ros2Config(
    "test_shape",
    [
        Ros2Publisher("test2", Int64)
    ],
    [
        Ros2Subscription("recv", "test_func", "payload_ref", String),
        Ros2Subscription("recv2", "test_func", "payload_ref", Int64),
    ]
)


def main():
    with Ros2Interface(cfg) as ros:
        while True:
            msg = input("Messaggio da inviare: ")
            if msg == "unsub":
                ros.unsubscribe("recv")
            ros.publish("test2", int(msg))


if __name__ == "__main__":
    main()