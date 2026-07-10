from flask import request
from flask_socketio import SocketIO, emit, join_room, leave_room
from web_gcs.topic_registry import get_registry, update_topic_data
from web_gcs.serializer import serialize_websocket_payload
import time

# Thread-safe SocketIO instance
socketio = SocketIO()

# Stores last emitted data per topic for change-detection filtering
_last_emitted_data = {}

def get_room_client_count(room_name):
    """
    Returns the number of clients currently joined to a topic's room.
    Uses socketio internal manager to inspect room memberships.
    """
    try:
        namespace = '/'
        rooms = socketio.server.manager.rooms.get(namespace)
        if rooms and room_name in rooms:
            return len(rooms[room_name])
    except Exception:
        pass
    return 0

def start_websocket_worker(out_queue):
    """
    Drains the bridge Queue, updates registry, checks room counts (lazy subscribe),
    checks for modifications (change detection), and pushes updates.
    """
    def worker():
        print("[WebSocket Worker] Active and listening for bridge events...")
        while True:
            try:
                name, data, timestamp = out_queue.get()
            except Exception:
                time.sleep(0.01)
                continue
                
            # Update registry memory
            update_topic_data(name, data, timestamp)
            
            # Lazy Subscription check: only broadcast if there are listeners
            listeners = get_room_client_count(name)
            if listeners > 0:
                # Change detection check
                last_val = _last_emitted_data.get(name)
                if data != last_val:
                    _last_emitted_data[name] = data
                    payload = serialize_websocket_payload(name, data, timestamp)
                    socketio.emit("topic_update", payload, room=name)
                    
            out_queue.task_done()
            
    socketio.start_background_task(worker)

def init_websocket(app):
    """Initializes SocketIO application parameters."""
    socketio.init_app(app, cors_allowed_origins="*", async_mode="threading")

@socketio.on("join_topic")
def on_join(data):
    topic = data.get("topic")
    registry = get_registry()
    if topic in registry:
        join_room(topic)
        # Immediately send latest telemetry to avoid cold start latency
        cached = registry[topic]
        if cached["latest_data"] is not None:
            emit("topic_update", serialize_websocket_payload(
                topic, cached["latest_data"], cached["last_update"]
            ))
    else:
        emit("error", {"message": f"Topic '{topic}' is not allowed (not in registry)"})

@socketio.on("leave_topic")
def on_leave(data):
    topic = data.get("topic")
    registry = get_registry()
    if topic in registry:
        leave_room(topic)
