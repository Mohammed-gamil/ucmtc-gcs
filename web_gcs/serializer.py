import time
from web_gcs.topic_registry import get_registry

def serialize_topic_rest(entry) -> dict:
    """Formats a registry entry for the GET /api/topics endpoints."""
    return {
        "name": entry["name"],
        "type": entry["type"],
        "fields": entry["schema"] or [],
        "connection_state": entry["connection_state"],
        "latest_data": entry["latest_data"],
        "last_update": entry["last_update"]
    }

def serialize_all_topics_rest(registry) -> list:
    """Formats the entire registry list for the GET /api/topics endpoint."""
    return [serialize_topic_rest(entry) for entry in registry.values()]

def serialize_websocket_payload(name, data, timestamp) -> dict:
    """Formats a topic update for the Socket.IO broadcast stream."""
    return {
        "topic": name,
        "timestamp": timestamp,
        "data": data
    }

def generate_mjpeg_stream(topic_name):
    """
    Yields JPEG binary frames directly from the ROS2 CompressedImage raw message
    saved in the topic registry.
    """
    registry = get_registry()
    if topic_name not in registry:
        return
        
    last_frame_time = 0.0
    while True:
        entry = registry[topic_name]
        if entry["latest_raw"] is not None and entry["last_update"] > last_frame_time:
            last_frame_time = entry["last_update"]
            try:
                msg = entry["latest_raw"]
                msg_type = type(msg).__name__
                if msg_type == "CompressedImage":
                    jpeg_bytes = bytes(msg.data)
                elif msg_type == "Image":
                    import numpy as np
                    import cv2
                    img_data = bytes(msg.data)
                    img_np = np.frombuffer(img_data, dtype=np.uint8)
                    expected_size = msg.height * msg.width * 3
                    if len(img_np) >= expected_size:
                        img_np = img_np[:expected_size].reshape((msg.height, msg.width, 3))
                        img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
                        _, jpeg_bytes_np = cv2.imencode('.jpg', img_bgr)
                        jpeg_bytes = bytes(jpeg_bytes_np)
                    else:
                        raise ValueError(f"Buffer size {len(img_np)} is less than expected {expected_size}")
                else:
                    jpeg_bytes = bytes(msg.data)
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n'
                       b'Content-Length: ' + str(len(jpeg_bytes)).encode('ascii') + b'\r\n\r\n' +
                       jpeg_bytes + b'\r\n')
            except Exception as e:
                print(f"[ERROR] Failed to extract JPEG bytes for binary stream {topic_name}: {e}")
        time.sleep(0.033)  # check at ~30Hz
