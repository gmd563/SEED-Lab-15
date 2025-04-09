import cv2
import numpy as np

def adjust_intensity(image, alpha=1.0, beta=0):
    new_image = np.clip(alpha * image + beta, 0, 255).astype(np.uint8)
    return new_image

image = cv2.imread('input.jpg')
adjusted_image = adjust_intensity(image, alpha=1.5, beta=20) #Increase contrast and brightness
cv2.imwrite('output.jpg', adjusted_image)
