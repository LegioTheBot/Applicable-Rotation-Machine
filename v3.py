import cv2
import numpy as np
import requests
from ultralytics import YOLO
import time
import serial

offset = 12 #offset that is quite literally making sure our arm is facing straight ahead 

arduino = serial.Serial("COM6", 115200, timeout=0.1, write_timeout=0.1)

def balance_white(img):   #This is purely a filter to get the camera feed semi-normal
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




# === Parameters for distance estimation ===
KNOWN_WIDTH_CM = 2.30     # Real object width (adjust as needed)
FOCAL_LENGTH = 525.84     # Calibrated focal length

frame_count = 0
# === Load YOLO model ===
model = YOLO(r"D:\robotic-arm-cv\runs\detect\train\weights\best.pt")  # Update path

def estimate_distance(known_width, perceived_width, focal_length):
    if perceived_width == 0:
        return None
    return (known_width * focal_length) / perceived_width

# === Start webcam ===
cap = cv2.VideoCapture(0)

#a clamped linear mapping
def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

# === Tracking variables for stationary object logic ===
last_pos = None
stationary_since = None
stationary_threshold = 3.0  # seconds
stationary_tolerance_px = 15  # px
stationary_label = "Yes"

# Before the while loop, add these:
base_angle = 90
j1_angle = 90
j2_angle = 90
grip = 90


while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Frame read failed")
        continue
    frame = balance_white(frame) #function to get the frame (video feed) thru the filter
    frame_count += 1
    height, width, _ = frame.shape
    center_frame_x = width // 2
    center_frame_y = height // 2

    
    # Default labels
    offset_label = "Offset from center: dx=N/A, dy=N/A"
    distance_label = "Distance: N/A"

    # Run YOLO detection
    results = model.predict(source=frame, imgsz=320, conf=0.5, verbose=False)
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
        center_y = ((y1 + y2) // 2 )
        cv2.circle(frame, (center_x, center_y), 5, (255, 255, 0), -1)

        # === Track stationary status ===
        current_time = time.time()
        if last_pos is not None:
            dx_static = abs(center_x - last_pos[0])
            dy_static = abs(center_y - last_pos[1])
            if dx_static < stationary_tolerance_px and dy_static < stationary_tolerance_px:
                if stationary_since is None:
                    stationary_since = current_time
            else:
                stationary_since = None
        last_pos = (center_x, center_y)


        # Offset from screen center
        dx = center_x - center_frame_x
        dy = center_y - center_frame_y
        #offset_label = f"Offset from center: dx={dx}px, dy={dy}px"


        if width_in_px > 0:
            cm_per_pixel = KNOWN_WIDTH_CM / width_in_px
            dx_cm = -dx * cm_per_pixel + offset
            dy_cm = dy * cm_per_pixel
            offset_label = f"Offset from center: dx={dx_cm:.2f} cm, dy={dy_cm:.2f} cm"


            # === Convert offsets to servo angles ===
            base_angle = clamp(int(dx_cm * 10.0 - 10), 0, 180)  #Multiplying it by 10 scales the vertical movement into a usable servo angle adjustment range.
            #dx'i 10 ile çarptığınızda dikey hareket, kullanılabilir bir servo açı ayar aralığına ölçeklenir.
            #dx_cm > 0: object is to the right of the center
            #Subtract 10 for offset correction (fine-tuning what the servo considers "centered")
            j1_angle = clamp(int((120 - (dy_cm * 10)) - 30), 0, 180) #120 : The angle when the servo is at starting point
            j2_angle = clamp(int(90 + (dy_cm * 10)), 0, 180) 
            
            # === Trigger approach if object stationary for 3+ sec and still far ===
            # Check if object is stationary
            object_is_stationary = stationary_since and (current_time - stationary_since > stationary_threshold)

            # If stationary, start reaching forward
            if object_is_stationary:
                j1_angle += 3
                print(f"[INFO] Reaching forward, j1_angle: {j1_angle}")
          

                #Grip logic
            grip = 155 if (object_is_stationary and distance and distance < 10) else 90

            # === Send to Arduino ===
            command = f"#{base_angle},{j1_angle},{j2_angle},{grip}\n"
            try:
                if frame_count % 5 == 0:
                    arduino.write(command.encode())
                    time.sleep(0.01)

# Read response
                    while arduino.in_waiting:
                        line = arduino.readline().decode().strip()
                        print("Arduino:", line)
            except serial.SerialTimeoutException:
                print("Serial write timeout — skipping frame")
            except Exception as e:
                print(f"Serial error: {e}")


        # Offset label under box
        cv2.putText(frame, offset_label, (x1, y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 255), 2)

        # Draw center lines with gaps over object
        cv2.line(frame, (center_frame_x, 0), (center_frame_x, y1), (200, 200, 200), 1)
        cv2.line(frame, (center_frame_x, y2), (center_frame_x, height), (200, 200, 200), 1)
        cv2.line(frame, (0, center_frame_y), (x1, center_frame_y), (200, 200, 200), 1)
        cv2.line(frame, (x2, center_frame_y), (width, center_frame_y), (200, 200, 200), 1)

        cv2.putText(frame, f"{object_is_stationary}", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 255), 2)
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