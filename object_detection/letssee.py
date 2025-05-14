import os
from ultralytics import YOLO
from multiprocessing import freeze_support

# Enable synchronous CUDA for debugging
os.environ['CUDA_LAUNCH_BLOCKING'] = '1'

def main():
    # Load the model
    model = YOLO("yolo11n.pt")  # Use a smaller model if needed

    # Train the model
    train_results = model.train(
        data=r"C:\Users\User\Desktop\robotic-arm-cv\OBJECT_DETECT\data.yaml",
        epochs=50,
        imgsz=320,  # Reduce image size
        batch=8,    # Reduce batch size
        device=0    # Use GPU 0
    )

if __name__ == '__main__':
    freeze_support()  # Required for multiprocessing on Windows
    main()