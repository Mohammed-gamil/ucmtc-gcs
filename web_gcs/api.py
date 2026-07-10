from flask import Blueprint, jsonify, Response, abort
from web_gcs.topic_registry import get_registry
from web_gcs.serializer import serialize_all_topics_rest, serialize_topic_rest, generate_mjpeg_stream

api_bp = Blueprint("api_bp", __name__)

@api_bp.route("/api/topics", methods=["GET"])
def get_topics():
    """Returns a full list of all topics in the registry with their cached schemas and status."""
    registry = get_registry()
    return jsonify(serialize_all_topics_rest(registry))

@api_bp.route("/api/topics/<path:topic_name>", methods=["GET"])
def get_topic(topic_name):
    """Returns status and schema of a specific topic, enforcing closed-list security."""
    full_name = topic_name
    if not full_name.startswith("/"):
        full_name = "/" + full_name
        
    registry = get_registry()
    if full_name not in registry:
        return jsonify({"error": "Topic not found"}), 404
        
    return jsonify(serialize_topic_rest(registry[full_name]))

@api_bp.route("/api/topics/<path:topic_name>/stream", methods=["GET"])
def get_topic_stream(topic_name):
    """Provides a live MJPEG stream for binary topics like cameraCompressedImage."""
    full_name = topic_name
    if not full_name.startswith("/"):
        full_name = "/" + full_name
        
    registry = get_registry()
    if full_name not in registry:
        abort(404)
        
    entry = registry[full_name]
    if entry.get("transport") != "binary_stream":
        return jsonify({"error": "Specified topic is not a binary_stream transport"}), 400
        
    return Response(
        generate_mjpeg_stream(full_name),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )
