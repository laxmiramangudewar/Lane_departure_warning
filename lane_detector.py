import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model

class LaneDetector:
    """
    Lane detection using a trained CNN (e.g. U-Net/SCNN) and OpenCV Hough fallback.
    """
    def __init__(self, model_path):
        # Load the pre-trained lane segmentation model
        self.model = load_model(model_path)
        # Set image dimensions expected by model
        self.input_width = 512
        self.input_height = 256

    def preprocess(self, image):
        """
        Resize and normalize the image for the CNN.
        """
        img = cv2.resize(image, (self.input_width, self.input_height))
        img = img / 255.0  # normalize
        return img

    def segment_lanes(self, image):
        """
        Run the CNN to produce a binary lane mask.
        """
        img = self.preprocess(image)
        # Add batch dimension
        inp = np.expand_dims(img, axis=0)
        # Predict lane mask
        mask = self.model.predict(inp)[0,:,:,0]
        # Threshold to binary
        mask = (mask > 0.5).astype(np.uint8) * 255
        # Resize mask to original image size
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
        return mask

    def detect_edges(self, image):
        """
        Edge detection for fallback: apply Canny.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        return edges

    def detect_lines(self, mask):
        """
        Detect lane lines from the mask using Hough transform.
        """
        # Use probabilistic Hough transform
        lines = cv2.HoughLinesP(mask, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=30)
        line_img = np.zeros_like(mask)
        if lines is not None:
            for x1,y1,x2,y2 in lines[:,0]:
                cv2.line(line_img, (x1,y1), (x2,y2), 255, 5)
        return line_img, lines

    def get_lane_center_offset(self, lines, image_shape):
        """
        Estimate lane center offset from detected line segments.
        """
        if lines is None:
            return None  # no lane detected
        # Separate left and right lines by slope
        left_x = []
        right_x = []
        for x1,y1,x2,y2 in lines[:,0]:
            slope = (y2 - y1) / (x2 - x1 + 1e-6)
            if abs(slope) < 0.5:  # ignore horizontal lines
                continue
            if slope < 0:
                left_x.extend([x1, x2])
            else:
                right_x.extend([x1, x2])
        if not left_x or not right_x:
            return None
        # Average x positions
        left_avg = int(np.mean(left_x))
        right_avg = int(np.mean(right_x))
        lane_center = (left_avg + right_avg) // 2
        image_center = image_shape[1] // 2
        offset = lane_center - image_center
        return offset

    def process(self, image):
        """
        Full pipeline: segmentation, edge detection, line fitting, and compute offset.
        """
        mask = self.segment_lanes(image)
        # Combine mask and edges to handle missing paint
        edges = self.detect_edges(image)
        combined = cv2.bitwise_or(mask, edges)
        line_img, lines = self.detect_lines(combined)
        offset = self.get_lane_center_offset(lines, image.shape)
        return offset, mask, line_img
