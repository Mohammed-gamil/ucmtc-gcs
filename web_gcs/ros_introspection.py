import array
import base64
import math
import numpy as np
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

def get_message_class(type_str: str):
    """
    Resolves a ROS2 message class by its type name.
    Example: 'sensor_msgs/msg/BatteryState' or 'geometry_msgs/Vector3'
    """
    parts = type_str.split('/')
    if len(parts) == 2:
        # Standardize nested type references (e.g. geometry_msgs/Vector3 -> geometry_msgs/msg/Vector3)
        type_str = f"{parts[0]}/msg/{parts[1]}"
    return get_message(type_str)

def get_schema(type_name: str, visited=None) -> list:
    """
    Recursively builds a field schema dictionary for the message type.
    Returns: list of dicts with {name, type, fields (optional), is_array}
    """
    if visited is None:
        visited = set()
        
    if '/' not in type_name:
        return []
        
    if type_name in visited:
        return []
    visited.add(type_name)
    
    try:
        msg_class = get_message_class(type_name)
        fields = msg_class.get_fields_and_field_types()
        schema = []
        for name, ftype in fields.items():
            base_type = ftype
            is_array = False
            if '[' in ftype:
                base_type = ftype.split('[')[0]
                is_array = True
            elif ftype.startswith('sequence<'):
                base_type = ftype[len('sequence<'):-1]
                is_array = True
                
            if '/' in base_type:
                nested = get_schema(base_type, visited.copy())
                schema.append({
                    "name": name,
                    "type": ftype,
                    "fields": nested,
                    "is_array": is_array
                })
            else:
                schema.append({
                    "name": name,
                    "type": ftype,
                    "is_array": is_array
                })
        return schema
    except Exception as e:
        return [{"error": f"Failed to load schema: {str(e)}"}]

def make_json_safe(data):
    """
    Recursively scans and converts any non-serializable objects (numpy arrays,
    raw bytes, array.array) to JSON-serializable primitives.
    """
    if isinstance(data, dict):
        return {str(k): make_json_safe(v) for k, v in data.items()}
    elif isinstance(data, (list, tuple)):
        return [make_json_safe(x) for x in data]
    elif isinstance(data, (np.ndarray, array.array)):
        return make_json_safe(data.tolist())
    elif isinstance(data, (bytes, bytearray)):
        try:
            return data.decode('utf-8')
        except UnicodeDecodeError:
            return base64.b64encode(data).decode('utf-8')
    elif isinstance(data, (np.floating, float)):
        val = float(data)
        if math.isnan(val) or math.isinf(val):
            return None
        return val
    elif isinstance(data, (np.integer, int)):
        return int(data)
    elif isinstance(data, (str, bool)) or data is None:
        return data
    else:
        return str(data)

def message_to_dict(msg) -> dict:
    """
    Converts a live ROS message to a JSON-safe nested dictionary.
    """
    ordered_dict = message_to_ordereddict(msg)
    return make_json_safe(ordered_dict)

def flatten_dict(d: dict, parent_key: str = '', sep: str = '.') -> dict:
    """
    Recursively flattens a nested dictionary.
    e.g. {'linear': {'x': 1.0}} -> {'linear.x': 1.0}
    """
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)
