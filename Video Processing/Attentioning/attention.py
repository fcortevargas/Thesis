import cv2
import subprocess
from random import randint
import os

def extract_audio_from_video(input_video_path, output_audio_path):
    cmd = [
        'ffmpeg',
        '-i', input_video_path,
        '-vn', '-acodec', 'copy',
        output_audio_path
    ]
    subprocess.run(cmd)

def add_audio_to_video(input_video_path, input_audio_path, output_video_path):
    cmd = [
        'ffmpeg',
        '-i', input_video_path,
        '-i', input_audio_path,
        '-c:v', 'copy',
        '-c:a', 'aac',
        '-shortest',
        output_video_path
    ]
    subprocess.run(cmd)

def save_video_segment(frames, output_path, fps, frame_size, fourcc=cv2.VideoWriter_fourcc(*'mp4v')):
    """Saves a list of frames as a video segment."""
    out = cv2.VideoWriter(output_path, fourcc, fps, frame_size)
    for f in frames:
        out.write(f)
    out.release()

def attention_video(input_file_path, output_file_path, video_number):
    video_capture = cv2.VideoCapture(input_file_path)

    # Get video properties
    fps = video_capture.get(cv2.CAP_PROP_FPS)
    frame_count = int(video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Define the font and other parameters
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 2
    font_thickness = 4
    text_color = (0, 0, 0)  # black color
    text_position = (randint(500, 1420), randint(250, 830))  # (x, y) position of the text
    duration = 3 # amount of time that the number will show on the video
    starting_frame = randint(1, 15) * fps # first frame that the number will appear

    # List to hold frames for video segment
    frames = []

    # Loop through each frame of the video
    frame_number = 0
    while True:
        ret, frame = video_capture.read()
        if not ret:
            break  # Break the loop when the video ends
        
        # Add the frame number text
        if frame_number >= starting_frame and frame_number < starting_frame + duration * fps:
            frame = cv2.putText(frame, f'{video_number}', text_position, font, font_scale, text_color, font_thickness)
        
        frames.append(frame)
        
        frame_number += 1

    video_no_audio_path = 'video_no_audio.mp4'
    audio_extracted_path = 'audio_extracted.aac'

    # Save video segment with added text but no audio
    save_video_segment(frames, video_no_audio_path, fps, (width, height))

    # Extract audio from input video
    extract_audio_from_video(input_file_path, audio_extracted_path)

    # Combine video and audio
    add_audio_to_video(video_no_audio_path, audio_extracted_path, output_file_path)

    # Release video capture and writer objects
    video_capture.release()

    # Clean up temporary files
    subprocess.run(['rm', video_no_audio_path])
    subprocess.run(['rm', audio_extracted_path])

def attention_videos_in_folder(input_folder, output_folder):
    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Iterate through all files in the input folder
    for filename in os.listdir(input_folder):
        if filename.endswith(".mp4"):
            video_number = int(filename.split("_")[0])
            if video_number % 4 == 0:
                input_file_path = os.path.join(input_folder, filename)
                output_file_path = os.path.join(output_folder, filename)
                attention_video(input_file_path, output_file_path, video_number)

attention_videos_in_folder('../Videos/Processed/Side', '../Videos/Processed/Attention/Side')
