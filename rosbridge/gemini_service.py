#!/usr/bin/env python3
# This file runs in your Python 3.10 environment
from flask import Flask, request, jsonify
import os
import json
import sys
import numpy as np
from PIL import Image
import io
import base64
from google import genai
from google.genai import types

# Import the Solution class from od.py
sys.path.append('/home/er/Documents/reasoning320/googleResoning/rosbridge')
from od import Solution

app = Flask(__name__)
object_detector = Solution()

@app.route('/detect_object', methods=['POST'])
def detect_object():
    """API endpoint to detect objects in an image"""
    try:
        # Get image data from request
        data = request.json
        image_path = data.get('image_path')
        
        if not image_path or not os.path.exists(image_path):
            return jsonify({"error": f"Image not found at {image_path}"}), 404
        
        # Run object detection using the Solution class
        prompt = "Detect the 2d bounding boxes of only relay (with \"label\" as topping description)"
        
        # Load image
        with open(image_path, "rb") as f:
            image_bytes = f.read()
        
        # Create image part for the API
        image_part = {"mime_type": "image/png", "data": image_bytes}
        
        # Run model to find bounding boxes
        response = object_detector.client.models.generate_content(
            model=object_detector.model_name,
            contents=[
                {"role": "user", "parts": [{"text": prompt}]},
                {"role": "user", "parts": [{"inline_data": image_part}]}
            ],
            config=types.GenerateContentConfig(
                system_instruction=object_detector.bounding_box_system_instructions,
                temperature=0.5,
            )
        )
        
        # Parse the JSON response
        bounding_boxes = object_detector.parse_json(response.text)
        bboxes = json.loads(bounding_boxes)
        
        if not bboxes:
            return jsonify({"error": "No objects detected"}), 404
        
        # Get the first detected object's center coordinates
        bbox = bboxes[0]
        y1, x1, y2, x2 = bbox["box_2d"]
        
        # Load image to get dimensions
        im = Image.open(image_path)
        width, height = im.size
        
        # Convert normalized coordinates (0-1000) to pixel coordinates
        pixel_x1 = (x1 / 1000) * width
        pixel_y1 = (y1 / 1000) * height
        pixel_x2 = (x2 / 1000) * width
        pixel_y2 = (y2 / 1000) * height
        
        # Calculate center in pixel coordinates
        center_x = int((pixel_x1 + pixel_x2) / 2)
        center_y = int((pixel_y1 + pixel_y2) / 2)
        
        return jsonify({
            "center_x": center_x,
            "center_y": center_y,
            "label": bbox.get("label", "relay"),
            "bbox": [y1, x1, y2, x2]
        })
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)