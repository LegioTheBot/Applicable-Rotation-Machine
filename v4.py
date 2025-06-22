import cv2
import numpy as np
import time
import serial
import math
from ultralytics import YOLO

# === Serial connection to Arduino ===
arduino = serial.Serial("COM9", 115200, timeout=0.1, write_timeout=0.1)

# === Load YOLO model ===
model = YOLO(r"C:\Users\AM Directive Node\Documents\PlatformIO\Projects\Applicable Rotation Machine - Laptop\object_detection\runs\detect\train\weights\best.pt")  # Update this path

# === Real-world parameters for distance estimation ===
CENTER_OFFSET_Y = -50 # Negative is up, positive is down
KNOWN_WIDTH_CM = 2.30
FOCAL_LENGTH = 525.84
GRIPPER_OBJECT_Y_DISTANCE_CM = -2.0
J2_LENGTH_CM = 12.7

# === Video stream ===
cap = cv2.VideoCapture(1)

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
base_angle = 90.0
j1_angle = 90.0
j2_angle = 90.0
grip = 90

# === Servo gain tuning ===
dx_gain = 0.08
dy_gain = 0.2
dx_limit = 0.1
dy_limit = 0.5
j2_base = 90
distance_threshold_cm = 10
close_range_cm = 12.0
close_range_multiplier = 0.4

# === Stationary tracking logic ===
last_pos = None
stationary_since = None
stationary_threshold = 100.0  # seconds
stationary_tolerance_px = 15

frame_count = 0

# === State logic ===
is_state_grabbing = False
grab_state_angle = 0.0
grab_state_duration = 5.0 # seconds

def send_command():
    # === Send command to Arduino ===
    command = f"#{base_angle},{j1_angle},{j2_angle},{grip}\n"
    try:
        arduino.write(command.encode())
        time.sleep(0.01)
        while arduino.in_waiting:
            line = arduino.readline().decode().strip()
            print("Arduino:", line)
    except Exception as e:
        print(f"Serial error: {e}")


while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Frame read failed")
        continue

    frame = balance_white(frame)
    frame_count += 1
    height, width, _ = frame.shape
    center_frame_x =width // 2
    center_frame_y = (height // 2) + CENTER_OFFSET_Y

    # === Object detection ===
    results = model.predict(source=frame, imgsz=320, conf=0.5, verbose=False)
    boxes = results[0].boxes

    offset_label = "Offset: N/A"
    distance_label = "Distance: N/A"
    object_is_stationary = False

    current_time = time.time()

    if is_state_grabbing:
        # === Grabbing State ===
        enter_y = math.sin(math.radians(state_grab_angle - 90)) * J2_LENGTH_CM
        target_y = enter_y + GRIPPER_OBJECT_Y_DISTANCE_CM
        final_grab_angle = math.degrees(math.asin(target_y / J2_LENGTH_CM)) + 90

        if j2_angle != final_grab_angle and grip < 150:
            j2_angle += max(min(final_grab_angle - j2_angle, 1.0), -1.0)
            grab_time = current_time + grab_state_duration
        else:
            grip = 155
            if (grab_time - current_time) < 0.0:
                grip = 90
                send_command()
                is_state_grabbing = False
            # Return to previous j2 angle after grabbing for 2.0s
            elif (grab_time - current_time) < grab_state_duration - 2.0:
                j2_angle = state_grab_angle
        if frame_count % 5 == 0:
            send_command()

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

        if not is_state_grabbing:
            # === Tracking State ===
            # === Servo control (pixel-based) ===
            grip = 90

            tracking_tolerance = 10  # deadzone

            gain_multiplier = 1.0 if distance > close_range_cm else close_range_multiplier

            if abs(dx) > tracking_tolerance:
                base_gain = clamp(float(dx * dx_gain * gain_multiplier), -dx_limit, dx_limit)
                base_angle = clamp(base_angle - base_gain, 0.0, 180.0)
            # if abs(dy) > tracking_tolerance:
            #     j1_gain = clamp(float(-dy * dy_gain), -dy_limit, dy_limit)
            #     j1_angle = clamp(j1_angle - j1_gain, 20.0, 180.0)

            if abs(dy) > tracking_tolerance:
                j2_gain = clamp(float(dy * dy_gain * gain_multiplier), -dy_limit, dy_limit)
                j2_angle = clamp(j2_angle + j2_gain, 0.0, 180.0)

                #j2_angle = clamp(j2_base + float((dy) * 0.05), 0, 180)  #40, 140 are angle limits for j2 i found from old code, may be incorrect
            #else:
                #j2_angle = j2_base  #whoever added this else will be hung on a cross by me -efe

            if abs(dx) < 5 and abs(dy) < 5 and distance and distance < distance_threshold_cm:
                is_state_grabbing = True
                # Store j2 angle when entering grab state
                state_grab_angle = j2_angle

        if frame_count % 5 == 0:
            send_command()

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

    cv2.putText(frame, f"Grab State: {is_state_grabbing}", (50, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 255), 2)
    # Draw center marker
    cv2.circle(frame, (center_frame_x, center_frame_y), 5, (0, 255, 255), -1)
    cv2.imshow("Eye-in-Hand Arm Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
