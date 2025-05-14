
import cv2
from ultralytics import YOLO
import numpy as np
# === Parameters for distance estimation ===
KNOWN_WIDTH_CM = 2.30     # Real object width (adjust as needed)
FOCAL_LENGTH = 525.84     # Calibrated focal length

# === Load YOLO model ===
model = YOLO(r"D:\robotic-arm-cv\runs\detect\train\weights\best.pt")  # Update path

def estimate_distance(known_width, perceived_width, focal_length):
    if perceived_width == 0:
        return None
    return (known_width * focal_length) / perceived_width

# === Start webcam ===
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    center_frame_x = width // 2
    center_frame_y = height // 2

    # Default labels
    offset_label = "Offset from center: dx=N/A, dy=N/A"
    distance_label = "Distance: N/A"

    # Run YOLO detection
    results = model.predict(source=frame, imgsz=640, conf=0.5, verbose=False)
    boxes = results[0].boxes

    if boxes:
        # Process only the first object
        box = boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        class_id = int(box.cls[0])
        width_in_px = x2 - x1

        # Estimate distance
        distance = estimate_distance(KNOWN_WIDTH_CM, width_in_px, FOCAL_LENGTH)
        distance_label = f"{model.names[class_id]}: {distance:.1f} cm" if distance else "Distance: N/A"

        # Bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, distance_label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Object center
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        cv2.circle(frame, (center_x, center_y), 5, (255, 255, 0), -1)

        # Offset from screen center
        dx = center_x - center_frame_x
        dy = center_y - center_frame_y

        if width_in_px > 0:
            cm_per_pixel = KNOWN_WIDTH_CM / width_in_px
            dx_cm = dx * cm_per_pixel
            dy_cm = dy * cm_per_pixel
            offset_label = f"Offset from center: dx={dx_cm:.2f} cm, dy={dy_cm:.2f} cm"

        # Offset label under box
        cv2.putText(frame, offset_label, (x1, y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 255), 2)

        # Draw center lines with gaps over object
        cv2.line(frame, (center_frame_x, 0), (center_frame_x, y1), (200, 200, 200), 1)
        cv2.line(frame, (center_frame_x, y2), (center_frame_x, height), (200, 200, 200), 1)
        cv2.line(frame, (0, center_frame_y), (x1, center_frame_y), (200, 200, 200), 1)
        cv2.line(frame, (x2, center_frame_y), (width, center_frame_y), (200, 200, 200), 1)
    else:
        # No detection — show fallback lines and info
        cv2.line(frame, (center_frame_x, 0), (center_frame_x, height), (200, 200, 200), 1)
        cv2.line(frame, (0, center_frame_y), (width, center_frame_y), (200, 200, 200), 1)

        cv2.putText(frame, offset_label, (10, height - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, distance_label, (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Always draw frame center dot
    cv2.circle(frame, (center_frame_x, center_frame_y), 5, (0, 255, 255), -1)

    # Show output
    cv2.imshow("Detection + Distance + Offset", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()