import os
import serial
import time
from deepface import DeepFace
import cv2
from collections import Counter

# Tells code not to use GPU
os.environ["CUDA_VISIBLE_DEVICES"] = ""

# Load face cascade classifier
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Start capturing video
cap = cv2.VideoCapture(0)

# arduino = serial.Serial("/dev/ttyACM0", 9600)
# time.sleep(2)

# Tracking state
collecting = False
start_time = None
emotions = []

while True:
    ret, frame = cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rgb_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2RGB)

    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        face_roi = rgb_frame[y:y + h, x:x + w]

        # Perform emotion analysis
        result = DeepFace.analyze(face_roi, actions=['emotion'], enforce_detection=False)
        emotion = result[0]['dominant_emotion']

        # Save emotion if we’re collecting
        if collecting:
            emotions.append(emotion)

        # Draw rectangle + label
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, emotion, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

    # Show window
    cv2.imshow('Real-time Emotion Detection', frame)

    key = cv2.waitKey(1) & 0xFF

    # Press 's' to start 15-second collection
    if key == ord('s') and not collecting:
        collecting = True
        start_time = time.time()
        emotions = []
        print("Started collecting emotions for 15 seconds...")

    # Check if 15 seconds passed
    if collecting and (time.time() - start_time >= 15):
        collecting = False
        print("=== Emotion Analysis Results ===")
        if emotions:
            total = len(emotions)
            counts = Counter(emotions)
            for emo, count in counts.items():
                percentage = (count / total) * 100
                print(f"{emo}: {count} times ({percentage:.2f}%)")
            print(f"Total detections: {total}")
        else:
            print("No emotions detected in the 15-second window.")
        print("================================")

    # Press 'q' to quit
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
