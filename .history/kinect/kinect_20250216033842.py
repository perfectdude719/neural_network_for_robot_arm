import cv2
import numpy as np
from freenect import sync_get_video, sync_get_depth

# Capture RGB and depth data
rgb, _ = sync_get_video()
depth, _ = sync_get_depth()

# Convert RGB data to a format OpenCV can use
rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

# Display the RGB and depth images
cv2.imshow('RGB', rgb)
cv2.imshow('Depth', depth.astype(np.uint8))
cv2.waitKey(0)
cv2.destroyAllWindows()