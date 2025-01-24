
import cv2

from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO('/home/sakar1/agrobot_ws/src/agrobot_vision/agrobot_vision/model/cotton_plant_model.engine')

# Open the video file
cap = cv2.VideoCapture(0)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLO11 tracking on the frame, persisting tracks between frames
        results = model.track(frame, persist=True)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLO11 Tracking", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()

# from ultralytics import YOLO

# # Load a YOLO11n PyTorch model
# model = YOLO("/home/sakar1/agrobot_ws/src/agrobot_vision/agrobot_vision/model/cotton_plant_model.pt")

# # Export the model to TensorRT

# model.export(format="engine")  # creates 'yolo11n.engine'

# Load the exported TensorRT model
# trt_model = YOLO("yolo11n.engine")