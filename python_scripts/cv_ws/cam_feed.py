import cv2
import numpy as np


cap = cv2.VideoCapture(0)

def draw_grid(frame, x, y, levels):

    # print(frame.shape[0]) # 480
    # print(frame.shape[1]) # 640
    # ___________ y
    # |
    # |
    # |
    # |
    # x
    start_x = 245
    end_x = 400
    start_y = 250
    end_y = 375

    cv2.line(frame, (start_x, x), (start_x, end_x), color=(255, 0, 255), thickness=1)
    cv2.line(frame, (start_x+x, x), (start_x+x, end_x), color=(255, 0, 255), thickness=1)
    cv2.line(frame, (start_x+2*x, x), (start_x+2*x, end_x), color=(255, 0, 255), thickness=1)
    cv2.line(frame, (start_x+3*x, x), (start_x+3*x, end_x), color=(255, 0, 255), thickness=1)

    # for i in range(levels):

    cv2.line(frame, (start_y, y + end_y), (end_x, y + end_y), color=(255, 0, 255),thickness=1)




while True:
    
    ret, image = cap.read()
    
    # b & w
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # gaussian blur
    blur = cv2.GaussianBlur(gray, (5,5), 0)


    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    # cv2.imshow("thresh", thresh)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    for contour in contours:
        # Get bounding rectangle around the contour 
        x, y, w, h = cv2.boundingRect(contour)
        
        # Ensure the detected region is approximately 50x50 pixels (small "X" shape)
        if 30 < w < 60 and 30 < h < 60:
            aspect_ratio = w / float(h)  # Should be close to 1 for square-like "X"
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            compactness = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0  # Shape similarity score

            # Confidence Check: Ensure shape is square-like and compact
            confidence = (1.0 - abs(aspect_ratio - 1)) * compactness
            if confidence > 0.65:
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw green rectangle
                cv2.putText(image, f"X Detected ({confidence:.2f})", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


    cv2.imshow("Threshold", thresh)
    cv2.imshow("X Detection", image)



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
