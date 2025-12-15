import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tactigon_shapes.modules.ros2.extension import Ros2Process
from tactigon_shapes.modules.ros2.models import Ros2ShapeConfig, Ros2Publisher, Ros2Subscription, RosMessage

def save(msg: RosMessage):
    cv_image = bridge.imgmsg_to_cv2(msg.msg)
    cv2.imwrite("frame.png", cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR))

def show(msg: RosMessage):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg.msg)
        cv_image = cv2.resize(cv_image, (340, 280))
        cv2.imshow("Camera Frame", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

bridge = CvBridge()

cfg = Ros2ShapeConfig(
    "test_shape",
    [],
    [
        Ros2Subscription("/camera/image_raw", "", "", Image)
    ]
)

def main():
    with Ros2Process(cfg) as ros:
        while True:
            msg = ros.get_msg()
            if msg:
                show(msg)

if __name__ == "__main__":
    main()