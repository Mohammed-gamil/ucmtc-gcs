import json
import os
import time
from web_gcs.ros_introspection import get_message_class, get_schema

# Active registry holding topic state and schemas
REGISTRY = {}

def load_registry(topics_json_path=None):
    """
    Reads the fixed topics.json file, resolves ROS classes,
    caches schemas, and initializes topic state.
    """
    global REGISTRY
    if topics_json_path is None:
        topics_json_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "topics.json")
    
    if not os.path.exists(topics_json_path):
        print(f"[ERROR] topics.json not found at {topics_json_path}")
        return
        
    try:
        with open(topics_json_path, "r") as f:
            topics = json.load(f)
    except Exception as e:
        print(f"[ERROR] Failed to load topics.json: {e}")
        return
        
    for t in topics:
        name = t["name"]
        type_str = t["type"]
        transport = t.get("transport", "json")
        
        entry = {
            "name": name,
            "type": type_str,
            "transport": transport,
            "msg_class": None,
            "schema": None,
            "latest_data": None,
            "last_update": 0.0,
            "connection_state": "disconnected",
            "error_reason": None
        }
        
        # Resolve ROS2 message class and generate schema
        try:
            msg_class = get_message_class(type_str)
            entry["msg_class"] = msg_class
            entry["schema"] = get_schema(type_str)
        except Exception as e:
            # Degrade gracefully for uninstalled/unbuilt packages
            entry["connection_state"] = "error"
            entry["error_reason"] = f"Unknown or missing type definition: {str(e)}"
            # Keep msg_class and schema None
            
        REGISTRY[name] = entry

TELEMETRY_UPDATE_HOOK = None

def get_registry():
    return REGISTRY

def update_topic_data(name, data, timestamp=None):
    """Updates registry entry with latest incoming telemetry."""
    if name in REGISTRY:
        REGISTRY[name]["latest_data"] = data
        REGISTRY[name]["last_update"] = timestamp or time.time()
        if not REGISTRY[name]["connection_state"].startswith("error"):
            REGISTRY[name]["connection_state"] = "connected"
            
    if TELEMETRY_UPDATE_HOOK is not None:
        try:
            TELEMETRY_UPDATE_HOOK(name, data, timestamp)
        except Exception as e:
            pass

# Populate registry on load
load_registry()
