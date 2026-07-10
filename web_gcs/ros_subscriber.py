import threading
import queue
import time
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from web_gcs.topic_registry import get_registry, update_topic_data
from web_gcs.ros_introspection import message_to_dict

class RosBridge(threading.Thread):
    def __init__(self, node, out_queue: queue.Queue):
        super().__init__(daemon=True)
        self.node = node
        self.out_queue = out_queue
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)
        self.subscriptions = {}
        self._last_process_time = {}
        
        self._init_subscriptions()

    def _init_subscriptions(self):
        registry = get_registry()
        for name, entry in registry.items():
            msg_class = entry["msg_class"]
            if msg_class is None:
                # Errored topic (e.g., mtc_interfaces), skip subscription
                continue
                
            transport = entry.get("transport", "json")
            callback = self._make_callback(name, transport)
            
            try:
                sub = self.node.create_subscription(
                    msg_class,
                    name,
                    callback,
                    qos_profile_sensor_data
                )
                self.subscriptions[name] = sub
            except Exception as e:
                entry["connection_state"] = f"error: {str(e)}"
                print(f"[ROS Bridge] Failed to subscribe to {name}: {e}")

    def _make_callback(self, name, transport):
        def callback(msg):
            now = time.time()
            registry = get_registry()
            if name not in registry:
                return

            if transport == "binary_stream":
                # Bypass generic JSON pipeline completely
                registry[name]["latest_raw"] = msg
                registry[name]["last_update"] = now
                registry[name]["connection_state"] = "connected"
            else:
                # json or throttled_json
                if transport == "throttled_json":
                    last_t = self._last_process_time.get(name, 0.0)
                    # Cap processing at 2 Hz (min 0.5s interval)
                    if now - last_t < 0.5:
                        return
                    self._last_process_time[name] = now
                
                try:
                    data = message_to_dict(msg)
                    # Hand off to the Flask-SocketIO thread-safe queue
                    self.out_queue.put((name, data, now))
                except Exception as e:
                    print(f"[ERROR] Introspection failed on {name}: {e}")
                    
        return callback

    def run(self):
        print("[ROS Bridge] Spinning executor thread...")
        try:
            self.executor.spin()
        except Exception as e:
            print(f"[ROS Bridge] Executor exception: {e}")
