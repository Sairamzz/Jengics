import cv2
import numpy as np


def verify_parallel_edges(edges, approx):
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=10)

    if lines is None:
        return False

    angles = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
        angles.append(angle)

    # Sort angles into horizontal & vertical groups
    horizontal = [a for a in angles if -10 < a < 10 or 170 < a < 190]
    vertical = [a for a in angles if 80 < a < 100]

    return len(horizontal) >= 2 and len(vertical) >= 2  # Ensure at least 2 parallel lines


def detect_jenga_blocks(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Canny Edge Detection
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    jenga_blocks = []
    
    for cnt in contours:
        hull = cv2.convexHull(cnt)  # Get convex hull (better for distorted shapes)
        approx = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)  

        # Check if it's a quadrilateral (rectangle or trapezium)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            area = cv2.contourArea(approx)

            # Apply size filtering (Remove small/noisy contours)
            if 1000 < area < 20000:  
                # Apply Hough Line Transform to verify it's a structured block
                if verify_parallel_edges(edges, approx):
                    jenga_blocks.append((x, y, w, h))
                    cv2.polylines(frame, [approx], True, (0, 255, 0), 2)
                    cv2.putText(frame, "Jenga Block", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return frame, jenga_blocks


cap = cv2.VideoCapture(2)  # Change to 3 if needed

while True:
    ret, frame = cap.read()
    if not ret:
        break

    processed_frame, jenga_blocks = detect_jenga_blocks(frame)

    # Display the result
    cv2.imshow("Jenga Detection (Stable)", processed_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
