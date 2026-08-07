import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SENSOR_TOPIC = '/lines/line_1/sensors/sensor_1'
ANALYZE_TOPIC = '/lines/line_1/analyze'

DEFAULT_SENSOR = 'new_prediction'
DEFAULT_ANALYZE = 'Analyze last batch of data and give me the results'


class TestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self._sensor_pub = self.create_publisher(String, SENSOR_TOPIC, 10)
        self._analyze_pub = self.create_publisher(String, ANALYZE_TOPIC, 10)

    def publish_sensor(self, data: str):
        msg = String()
        msg.data = data
        self._sensor_pub.publish(msg)
        self.get_logger().info(f'[{SENSOR_TOPIC}] {data}')

    def publish_analyze(self, data: str):
        msg = String()
        msg.data = data
        self._analyze_pub.publish(msg)
        self.get_logger().info(f'[{ANALYZE_TOPIC}] {data}')


class App(tk.Tk):
    def __init__(self, node: TestPublisherNode):
        super().__init__()
        self._node = node
        self.title('ROS2 Test Publisher')
        self.resizable(False, False)
        self._build_ui()

    def _build_ui(self):
        pad = {'padx': 10, 'pady': 6}

        # ── Sensor topic ────────────────────────────────────────────────
        sensor_frame = ttk.LabelFrame(self, text=f'Topic: {SENSOR_TOPIC}')
        sensor_frame.pack(fill='x', **pad)

        self._sensor_var = tk.StringVar(value=DEFAULT_SENSOR)
        ttk.Entry(sensor_frame, textvariable=self._sensor_var, width=60).pack(
            side='left', padx=6, pady=6, fill='x', expand=True
        )
        ttk.Button(sensor_frame, text='Publish', command=self._on_sensor).pack(
            side='right', padx=6, pady=6
        )

        # ── Analyze topic ────────────────────────────────────────────────
        analyze_frame = ttk.LabelFrame(self, text=f'Topic: {ANALYZE_TOPIC}')
        analyze_frame.pack(fill='x', **pad)

        self._analyze_var = tk.StringVar(value=DEFAULT_ANALYZE)
        ttk.Entry(analyze_frame, textvariable=self._analyze_var, width=60).pack(
            side='left', padx=6, pady=6, fill='x', expand=True
        )
        ttk.Button(analyze_frame, text='Publish', command=self._on_analyze).pack(
            side='right', padx=6, pady=6
        )

        # ── Log ─────────────────────────────────────────────────────────
        log_frame = ttk.LabelFrame(self, text='Log')
        log_frame.pack(fill='both', expand=True, **pad)

        self._log = tk.Text(log_frame, height=8, state='disabled', wrap='word')
        self._log.pack(fill='both', expand=True, padx=4, pady=4)

    def _log_message(self, topic: str, data: str):
        self._log.configure(state='normal')
        self._log.insert('end', f'[{topic}]  {data}\n')
        self._log.see('end')
        self._log.configure(state='disabled')

    def _on_sensor(self):
        data = self._sensor_var.get().strip()
        if not data:
            messagebox.showwarning('Empty message', 'Inserisci un messaggio da pubblicare.')
            return
        self._node.publish_sensor(data)
        self._log_message(SENSOR_TOPIC, data)

    def _on_analyze(self):
        data = self._analyze_var.get().strip()
        if not data:
            messagebox.showwarning('Empty message', 'Inserisci un messaggio da pubblicare.')
            return
        self._node.publish_analyze(data)
        self._log_message(ANALYZE_TOPIC, data)


def main():
    rclpy.init()
    node = TestPublisherNode()

    # Spin rclpy in a background thread so the GUI stays responsive
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = App(node)
    app.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
