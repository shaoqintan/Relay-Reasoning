from google import genai
from google.genai import types
import os
import json
import random
import io
from PIL import Image, ImageDraw, ImageFont
from PIL import ImageColor
from PIL import Image
import requests
from io import BytesIO

from dotenv import load_dotenv

load_dotenv()  # take environment variables

# GOOGLE_API_KEY = os.getenv('GOOGLE_API_KEY')
GOOGLE_API_KEY = 'AIzaSyBUZrmRo-xQ5de1-S2nKqOwwW6sDHHC2qE'
client = genai.Client(api_key=GOOGLE_API_KEY)


class Solution:
    def __init__(self):
        self.client = genai.Client(api_key=GOOGLE_API_KEY)
        self.model_name = "gemini-2.0-flash"
        self.bounding_box_system_instructions = """
            Return bounding boxes as a JSON array with labels. Never return masks or code fencing. Limit to 25 objects.
            If an object is present multiple times, name them according to their unique characteristic (colors, size, position, unique characteristics, etc..).
        """

        self.additional_colors = [colorname for (colorname, colorcode) in ImageColor.colormap.items()]

    # @title Parsing JSON output
    def parse_json(self, json_output):
        # Parsing out the markdown fencing
        lines = json_output.splitlines()
        for i, line in enumerate(lines):
            if line == "```json":
                json_output = "\n".join(lines[i + 1:])  # Remove everything before "```json"
                json_output = json_output.split("```")[0]  # Remove everything after the closing "```"
                break  # Exit the loop once "```json" is found
        return json_output

    def plot_bounding_boxes(self, im, bounding_boxes):
        """
        Plots bounding boxes on an image with markers for each a name, using PIL, normalized coordinates, and different colors.

        Args:
            img_path: The path to the image file.
            bounding_boxes: A list of bounding boxes containing the name of the object
            and their positions in normalized [y1 x1 y2 x2] format.
        """

        # Load the image
        img = im
        width, height = img.size
        print(img.size)
        # Create a drawing object
        draw = ImageDraw.Draw(img)

        # Define a list of colors
        colors = [
                     'red',
                     'green',
                     'blue',
                     'yellow',
                     'orange',
                     'pink',
                     'purple',
                     'brown',
                     'gray',
                     'beige',
                     'turquoise',
                     'cyan',
                     'magenta',
                     'lime',
                     'navy',
                     'maroon',
                     'teal',
                     'olive',
                     'coral',
                     'lavender',
                     'violet',
                     'gold',
                     'silver',
                 ] + self.additional_colors

        # Parsing out the markdown fencing
        bounding_boxes = self.parse_json(bounding_boxes)

        font = ImageFont.truetype("NotoSansCJK-Regular.ttc", size=14)

        # Iterate over the bounding boxes
        for i, bounding_box in enumerate(json.loads(bounding_boxes)):
            # Select a color from the list
            color = colors[i % len(colors)]

            # Convert normalized coordinates to absolute coordinates
            abs_y1 = int(bounding_box["box_2d"][0] / 1000 * height)
            abs_x1 = int(bounding_box["box_2d"][1] / 1000 * width)
            abs_y2 = int(bounding_box["box_2d"][2] / 1000 * height)
            abs_x2 = int(bounding_box["box_2d"][3] / 1000 * width)

            if abs_x1 > abs_x2:
                abs_x1, abs_x2 = abs_x2, abs_x1

            if abs_y1 > abs_y2:
                abs_y1, abs_y2 = abs_y2, abs_y1

            # Draw the bounding box
            draw.rectangle(
                ((abs_x1, abs_y1), (abs_x2, abs_y2)), outline=color, width=4
            )

            # Draw the text
            if "label" in bounding_box:
                draw.text((abs_x1 + 8, abs_y1 + 6), bounding_box["label"], fill=color, font=font)

        # Display the image
        img.show()

    def relay_sorting(self):
        image_path = "/home/er/Documents/rostransform/current_frame.jpg"

        prompt = "Detect the 2d bounding boxes of only relay (with \"label\" as topping description)"

        # Load image
        with open(image_path, "rb") as f:
            image_bytes = f.read()
        
        # Create image part for the API
        image_part = {"mime_type": "image/png", "data": image_bytes}
        
        # Run model to find bounding boxes
        response = self.client.models.generate_content(
            model=self.model_name,
            contents=[
                {"role": "user", "parts": [{"text": prompt}]},
                {"role": "user", "parts": [{"inline_data": image_part}]}
            ],
            config=types.GenerateContentConfig(
                system_instruction=self.bounding_box_system_instructions,
                temperature=0.5,
            )
        )

        # Print raw response
        print(response.text)
        
        # Parse the JSON response
        bounding_boxes = self.parse_json(response.text)
        bboxes = json.loads(bounding_boxes)
        
        # Load image to get actual dimensions
        im = Image.open(image_path)
        width, height = im.size
        print(f"Actual image size: {width}x{height}")
        
        # Calculate and print center coordinates
        for bbox in bboxes:
            # The box_2d format is [y1, x1, y2, x2] normalized to 0-1000
            y1, x1, y2, x2 = bbox["box_2d"]
            
            # Print normalized coordinates for reference
            print(f"Normalized bbox coordinates: y1={y1}, x1={x1}, y2={y2}, x2={x2}")
            
            # Convert normalized coordinates (0-1000) to pixel coordinates
            pixel_x1 = (x1 / 1000) * width
            pixel_y1 = (y1 / 1000) * height
            pixel_x2 = (x2 / 1000) * width
            pixel_y2 = (y2 / 1000) * height
            
            # Calculate center in pixel coordinates
            center_x = (pixel_x1 + pixel_x2) / 2
            center_y = (pixel_y1 + pixel_y2) / 2
            
            print(f"Object: {bbox['label']}")
            print(f"Pixel coordinates: x1={pixel_x1:.1f}, y1={pixel_y1:.1f}, x2={pixel_x2:.1f}, y2={pixel_y2:.1f}")
            print(f"Center coordinates (pixels): ({center_x:.1f}, {center_y:.1f})")


if __name__ == '__main__':
    ob = Solution()
    ob.relay_sorting()