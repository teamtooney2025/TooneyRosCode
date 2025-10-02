#check and see if this will run on Pi
import os
import serial
import time
from deepface import DeepFace
from rclpy.node import Node
import cv2

from std_msgs.msg import String

#Tells code not to use GPU (for future expansion, unless continue with laptops)
os.environ["CUDA_VISIBLE_DEVICES"] = ""


# Load face cascade classifier
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Start capturing video
cap = cv2.VideoCapture(0)

arduino = serial.Serial("/dev/ttyACM0", 9600)
time.sleep(2)

# class Expression_detection():

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Convert grayscale frame to RGB format
    rgb_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2RGB)

    # Detect faces in the frame
    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        # Extract the face ROI (Region of Interest)
        face_roi = rgb_frame[y:y + h, x:x + w]

        
        # Perform emotion analysis on the face ROI
        result = DeepFace.analyze(face_roi, actions=['emotion'], enforce_detection=False)

        # Determine the dominant emotion
        emotion = result[0]['dominant_emotion']

        emotion_map = { #splits up the emotion messages recieved as a string, and then turns them into a number, to be turned into an integer
        "happy": 1,
        "sad": 2,
        "fear": 3,
        "surprise": 4,
        "neutral": 5,
        "angry": 6,
        "disgust": 7
        }

        emotion_id = emotion_map[emotion.lower()]


        print(emotion, emotion_id)
            
        arduino.write(bytes([emotion_id]))

        # Draw rectangle around face and label with predicted emotion
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, emotion, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)


    # Display the resulting frame
    cv2.imshow('Real-time Emotion Detection', frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close all windows
cap.release()
cv2.destroyAllWindows()


# class ExpressionPublisher(Node):

#     def __init__(self):
#         super().__init__('expression_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = String()
#         msg.data = emotion
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1


# def main(args=None):
#     rclpy.init(args=args)

#     expression_publisher = ExpressionPublisher()

#     rclpy.spin(expression_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     expression_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()