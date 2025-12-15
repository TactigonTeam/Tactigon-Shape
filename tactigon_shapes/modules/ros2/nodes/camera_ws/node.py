import rclpy
import os
import signal
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import asyncio
import base64
from threading import Thread
from socketio import Server, WSGIApp
from gevent.pywsgi import WSGIServer
from geventwebsocket.handler import WebSocketHandler
from multiprocessing import Process, Event
from multiprocessing.synchronize import Event as EventType

from tactigon_shapes.modules.ros2.nodes.camera_ws.models import CameraWsConfig

class CameraWsNode(Node):
    wsgi_server: WSGIServer
    sio: Server
    app: WSGIApp
    thread: Thread
    
    def __init__(self, topic: str, sio: Server):
        Node.__init__(self, CameraWsNode.__name__)
        self.sio = sio
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, topic, self.on_image, 10)

    def on_image(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
            self.sio.emit('frame', {'image': base64.b64encode(buffer).decode('utf-8')})

        except Exception as e:
            self.get_logger().error(f"Errore nella conversione o invio immagine: {e}")

    def destroy_node(self):
        self.destroy_subscription(self.subscription)
        Node.destroy_node(self)

class CameraWsThread(Thread):
    topic: str
    node: None | CameraWsNode

    def __init__(self, topic: str, sio: Server):
        Thread.__init__(self)
        self.topic = topic
        self.sio = sio
        self.node = None

    def run(self):
        rclpy.init()
        self.node = CameraWsNode(self.topic, self.sio)
        try:
            rclpy.spin(self.node)
        finally:
            rclpy.shutdown()

    def stop(self):
        if self.node is not None:
            self.node.destroy_node()


class CameraWs(Process):
    config: CameraWsConfig

    def __init__(self, config: CameraWsConfig):
        Process.__init__(self)
        self.config = config

    def run(self):
        sio = Server(async_mode="gevent", cors_allowed_origins="*")
        app = WSGIApp(sio)
        wsgi_server = WSGIServer((self.config.url, self.config.port), app, handler_class=WebSocketHandler)
        camera_thread = CameraWsThread(self.config.topic, sio)

        def graceful_shutdown(signum=None, frame=None):
            camera_thread.stop()
            camera_thread.join()
            wsgi_server.stop(timeout=5)
        
        signal.signal(signal.SIGTERM, graceful_shutdown)
        signal.signal(signal.SIGINT, graceful_shutdown)

        try:
            camera_thread.start()
            wsgi_server.serve_forever()
        finally:
            graceful_shutdown()

    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, *args, **kwargs):
        self.stop()


    def stop(self):
        pid = self.pid
        
        if pid:
            os.kill(pid, signal.SIGTERM)
            self.join()