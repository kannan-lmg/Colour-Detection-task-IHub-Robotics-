import cv2
import serial
import time

# Set up serial communication with Arduino
arduino = serial.Serial('COM3', 9600)  # ⚠️ Change 'COM3' to your actual Arduino port
time.sleep(2)  # Give Arduino time to reset

# Start webcam
cap = cv2.VideoCapture(0)

# Load OpenCV's built-in face detector
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Track last sent signal
last_sent = None

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Convert frame to grayscale for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

    if len(faces) > 0:
        print("Face detected!")
        if last_sent != '1':
            arduino.write(b'1')  # Turn LED ON
            last_sent = '1'

        # Draw boxes around faces
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    else:
        print("No face detected.")
        if last_sent != '0':
            arduino.write(b'0')  # Turn LED OFF
            last_sent = '0'

    # Show the video feed with rectangles
    cv2.imshow('Face Detection (OpenCV)', frame)

    # Break loop with 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
arduino.close()
