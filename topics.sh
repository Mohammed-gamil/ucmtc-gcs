#!/bin/bash
# Sourcing this script or running it outputs the static JSON topic list for the GCS.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
topics_file="$SCRIPT_DIR/web_gcs/topics.json"

if [ -f "$topics_file" ]; then
    cat "$topics_file"
else
    echo "[]"
fi