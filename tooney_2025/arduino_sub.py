# #This is a subscriber, and will publish messages for the Arduino to read. it will also receive Arduino messages maybe

# arduino = serial.Serial("/dev/ttyACM0", 9600)
# time.sleep(2)

# emotion_map = { #splits up the emotion messages recieved as a string, and then turns them into a number, to be turned into an integer
# "happy": 1,
# "sad": 2,
# "fear": 3,
# "surprise": 4,
# "neutral": 5,
# "angry": 6,
# "disgust": 7
# }

# emotion_id = emotion_map[emotion.lower()]


# print(emotion, emotion_id)
        
# arduino.write(bytes([emotion_id]))

