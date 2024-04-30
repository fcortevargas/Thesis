import cv2
import os

def extract_frames(video_path, output_folder, start_time, end_time, frame_rate=1):
    # Open the video file
    video_capture = cv2.VideoCapture(video_path)
    
    # Get video properties
    frame_count = int(video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = int(video_capture.get(cv2.CAP_PROP_FPS))
    
    # Calculate start and end frame
    start_frame = int(start_time * fps)
    end_frame = min(int(end_time * fps), frame_count)

    print(f"start_frame: {start_frame}")
    print(f"end_frame: {end_frame}")
    
    # Set the video capture to start at the specified frame
    video_capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    
    # Iterate through frames and save them as images
    frame_number = start_frame
    while frame_number < end_frame:
        video_capture.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
        ret, frame = video_capture.read()
        if not ret:
            break
        # Save the frame as an image
        output_path = os.path.join(output_folder, f"frame_{frame_number}.jpg")
        cv2.imwrite(output_path, frame)
        
        # Move to the next frame
        frame_number += int(frame_rate)
    
    # Release the video capture
    video_capture.release()

# Example usage
video_path = "input_video.mp4"
output_folder = "frames"
start_time = 0  # Start time in seconds
end_time = 5    # End time in seconds
frame_rate = 1   # Save every frame_rate-th frame (1 for every frame)

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Call the function to extract frames
extract_frames(video_path, output_folder, start_time, end_time, frame_rate)

print("Frames extracted successfully!")