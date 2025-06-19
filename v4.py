import cv2
import numpy as np
import time
import serial
from ultralytics import YOLO

# === Serial connection to Arduino ===
arduino = serial.Serial("COM6", 115200, timeout=0.1, write_timeout=0.1)

# === Load YOLO model ===
model = YOLO(r"D:\robotic-arm-cv\runs\detect\train\weights\best.pt")  # Update this path

# === Real-world parameters for distance estimation ===
KNOWN_WIDTH_CM = 2.30
FOCAL_LENGTH = 525.84

# === Video stream ===
cap = cv2.VideoCapture(0)

# === Helper ===
def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

# === Optional: basic white balancing ===
def balance_white(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB) #Converts the input image from BGR (default OpenCV format) to LAB color space.
    #L: lightness (0–255), A: green–magenta color axis (128 is neutral), B: blue–yellow color axis (128 is neutral)
    #LAB because it separates lightness from color channels, making it easier to adjust color without affecting brightness
    avg_a = result[..., 1].mean() # entire A channel, mean(): computes the mean color deviation
    avg_b = result[..., 2].mean() # entire B channel
    result[..., 1] = result[..., 1] - ((avg_a - 128) * (result[..., 0] / 255.0) * 1.1)
    #(avg_a - 128) tells how far the color is from neutral (i.e., how “green” or “magenta” it is).
    #(result[..., 0] / 255.0) normalizes the lightness to 0–1 range (brighter pixels are more corrected).
    #* 1.1 adds a slight bias to make the correction more aggressive.
    result[..., 2] = result[..., 2] - ((avg_b - 128) * (result[..., 0] / 255.0) * 1.1)
    return cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    #Converts the corrected LAB image back to standard BGR format, ready to be displayed or processed by OpenCV.


# === Servo state ===
base_angle = 90
j1_angle = 90
j2_angle = 90
grip = 90

# === Servo gain tuning ===
dx_gain = 0.08
dy_gain = 0.1
j2_base = 90
distance_threshold_cm = 10

# === Stationary tracking logic ===
last_pos = None
stationary_since = None
stationary_threshold = 3.0  # seconds
stationary_tolerance_px = 15

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Frame read failed")
        continue

    frame = balance_white(frame)
    frame_count += 1
    height, width, _ = frame.shape
    center_frame_x = width // 2
    center_frame_y = height // 2

    # === Object detection ===
    results = model.predict(source=frame, imgsz=320, conf=0.5, verbose=False)
    boxes = results[0].boxes

    offset_label = "Offset: N/A"
    distance_label = "Distance: N/A"
    object_is_stationary = False

    if boxes:
        box = boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        class_id = int(box.cls[0])
        width_in_px = x2 - x1

        # Distance estimation
        distance = (KNOWN_WIDTH_CM * FOCAL_LENGTH) / width_in_px if width_in_px > 0 else None
        if distance:
            distance_label = f"{model.names[class_id]}: {distance:.1f} cm"

        # Object center
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        dx = center_x - center_frame_x
        dy = center_y - center_frame_y
        offset_label = f"Offset: dx={dx}px, dy={dy}px"

        # === Track stationary state ===
        current_time = time.time()
        if last_pos:
            dx_static = abs(center_x - last_pos[0])
            dy_static = abs(center_y - last_pos[1])
            if dx_static < stationary_tolerance_px and dy_static < stationary_tolerance_px:
                if not stationary_since:
                    stationary_since = current_time
            else:
                stationary_since = None
        last_pos = (center_x, center_y)

        object_is_stationary = stationary_since and (current_time - stationary_since > stationary_threshold)

        # === Servo control (pixel-based) ===
        tracking_tolerance = 10  # deadzone
        if abs(dx) > tracking_tolerance:
            base_angle = clamp(base_angle - int(dx * dx_gain), 0, 180)
        if abs(dy) > tracking_tolerance:
            j1_angle = clamp(j1_angle + int(-dy * dy_gain), 0, 180)

        if distance and distance > distance_threshold_cm:
            j2_angle = clamp(j2_base + int((15 - distance) * 2.0), 60, 140)
        else:
            j2_angle = j2_base

        # === Gripper control ===
        if abs(dx) < 5 and abs(dy) < 5 and distance and distance < distance_threshold_cm:
            grip = 155  # close
        else:
            grip = 90   # open

        # === Send command to Arduino ===
        command = f"#{base_angle},{j1_angle},{j2_angle},{grip}\n"
        try:
            if frame_count % 5 == 0:
                arduino.write(command.encode())
                time.sleep(0.01)
                while arduino.in_waiting:
                    line = arduino.readline().decode().strip()
                    print("Arduino:", line)
        except Exception as e:
            print(f"Serial error: {e}")

        # === Draw on frame ===
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (255, 255, 0), -1)
        cv2.putText(frame, distance_label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, offset_label, (x1, y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 255), 2)
        cv2.putText(frame, f"Stationary: {object_is_stationary}", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 255), 2)
    else:
        cv2.putText(frame, offset_label, (10, height - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, distance_label, (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Draw center marker
    cv2.circle(frame, (center_frame_x, center_frame_y), 5, (0, 255, 255), -1)
    cv2.imshow("Eye-in-Hand Arm Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
