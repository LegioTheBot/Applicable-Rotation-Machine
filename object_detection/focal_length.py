import cv2
from ultralytics import YOLO

# === CONFIG ===       
KNOWN_WIDTH_CM = 2.3            # Real-world width of calibration object
CALIBRATION_DISTANCE_CM = 20.0   # Distance you place object from camera

# === Load your trained model ===
model = YOLO(r"D:\robotic-arm-cv\runs\detect\train\weights\best.pt")

# === Global focal length (will be updated during calibration)
FOCAL_LENGTH = None

# === Distance estimation function ===
def estimate_distance(known_width, perceived_width, focal_length):
    if perceived_width == 0:
        return None
    return (known_width * focal_length) / perceived_width

# === Webcam stream ===
cap = cv2.VideoCapture(0)
calibrated = False

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(source=frame, imgsz=640, conf=0.5, verbose=False)
    boxes = results[0].boxes

    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        class_id = int(box.cls[0])
        width_px = x2 - x1

        # If calibrated, estimate distance
        if FOCAL_LENGTH:
            distance = estimate_distance(KNOWN_WIDTH_CM, width_px, FOCAL_LENGTH)
            label = f"{model.names[class_id]}: {distance:.1f} cm"
        else:
            label = f"{model.names[class_id]} (Press 'c' to calibrate)"

        # Draw box and label
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if not calibrated:
        cv2.putText(frame, "Place known object 50cm away and press 'c' to calibrate",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow("Detection + Distance", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c') and boxes:
        # Use the widest detected box for calibration
        widest_box = max(boxes, key=lambda b: b.xyxy[0][2] - b.xyxy[0][0])
        x1, _, x2, _ = map(int, widest_box.xyxy[0])
        width_in_pixels = x2 - x1
        FOCAL_LENGTH = (width_in_pixels * CALIBRATION_DISTANCE_CM) / KNOWN_WIDTH_CM
        print(f"[✓] Focal length calibrated: {FOCAL_LENGTH:.2f} pixels")
        calibrated = True

cap.release()
cv2.destroyAllWindows()
