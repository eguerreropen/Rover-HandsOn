import time
from ultralytics import YOLO

# Load exported NCNN model once
model = YOLO("yolo26n_ncnn_model", task="detect")

image_path = "bus.jpg"
times = []

for i in range(20):
    start = time.perf_counter()
    results = model(image_path, verbose=False)
    end = time.perf_counter()

    elapsed = end - start
    times.append(elapsed)

    boxes = results[0].boxes
    print(f"Run {i+1:02d}: {elapsed:.4f} s | detections: {len(boxes)}")

avg = sum(times) / len(times)
print(f"\nAverage time: {avg:.4f} s")
print(f"Approx FPS: {1/avg:.2f}")
