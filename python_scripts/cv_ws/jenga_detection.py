import cv2
import numpy as np

# Initialize webcam
cap = cv2.VideoCapture(0)

while True:
    ret, image = cap.read()
    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian Blur to reduce noise
    blur = cv2.GaussianBlur(gray, (5,5), 0)

    # Adaptive Thresholding (Inverted for better detection)
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY_INV, 11, 2)

    # Find contours
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

    for contour in contours:
        # Get bounding rectangle around the contour
        x, y, w, h = cv2.boundingRect(contour)
        if w < 20 or h < 20 and w > 30 or h > 30:
            continue
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)


        

    # Display results
    cv2.imshow("Threshold", thresh)
    cv2.imshow("X Detection", image)

    # Exit on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
