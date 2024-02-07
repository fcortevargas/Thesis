import cv2
import subprocess
import numpy as np

def is_magenta_present(frame, threshold=50):
    """Check if there is a significant amount of magenta in the frame."""
    # Convert frame to RGB (OpenCV uses BGR by default)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    magenta_lower = np.array([255 - threshold, 0, 255 - threshold])
    magenta_upper = np.array([255, threshold, 255])
    mask = cv2.inRange(frame_rgb, magenta_lower, magenta_upper)
    return np.sum(mask) > 0

def extract_audio_from_video(input_video_path, output_audio_path):
    cmd = [
        'ffmpeg',
        '-i', input_video_path,
        '-vn', '-acodec', 'copy',
        output_audio_path
    ]
    subprocess.run(cmd)

def trim_audio(input_audio_path, start_time, duration, output_audio_path):
    cmd = [
        'ffmpeg',
        '-ss', str(start_time),
        '-t', str(duration),
        '-i', input_audio_path,
        '-c:a', 'aac',
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

def split_input_video(input_video_path):
    # Extract audio from input audio
    extract_audio_from_video(input_video_path, 'Input/input_audio.aac')

    cap = cv2.VideoCapture(input_video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    frames_per_segment = 20 * fps  # 20 seconds of frames
    segment_start_frame = None
    current_segment_frames = []
    video_index = 1
    video_count = 1
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # For MP4 output

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_count = cap.get(cv2.CAP_PROP_POS_FRAMES)

        if is_magenta_present(frame):
            # If magenta is present, check if the current segment should be saved
            if segment_start_frame is not None and len(current_segment_frames) >= frames_per_segment:
                out = cv2.VideoWriter(f'Processing/Video/trimmed_video_{video_index}.mp4', fourcc, fps, (int(cap.get(3)), int(cap.get(4))))
                for f in current_segment_frames:
                    out.write(f)
                out.release()
                if video_index % 2 == 0:
                    input_audio_path = 'Input/input_audio.aac'
                    start_time = frame_count / fps - 20
                    print(f'Start time: {start_time}')
                    duration = 20
                    output_audio_path = f'Processing/Audio/trimmed_audio_{video_index}.mp4'
                    trim_audio(input_audio_path, start_time, duration, output_audio_path)

                    input_video_path = f'Processing/Video/trimmed_video_{video_index - 1}.mp4'
                    input_audio_path = f'Processing/Audio/trimmed_audio_{video_index}.mp4'
                    output_video_path = f'Output/output_video_{video_count}.mp4'
                    add_audio_to_video(input_video_path, input_audio_path, output_video_path)
                    video_count += 1
                video_index += 1
            # Reset for the next segment
            segment_start_frame = None
            current_segment_frames = []
        else:
            if segment_start_frame is None:
                segment_start_frame = frame_count
            current_segment_frames.append(frame)

            # Check if we've reached the end of a 20-second segment
            if frame_count - segment_start_frame >= frames_per_segment:
                out = cv2.VideoWriter(f'Processing/Video/trimmed_video_{video_index}.mp4', fourcc, fps, (int(cap.get(3)), int(cap.get(4))))
                for f in current_segment_frames:
                    out.write(f)
                out.release()
                if video_index % 2 == 0:
                    input_audio_path = 'Input/input_audio.aac'
                    start_time = frame_count / fps - 20
                    print(f'Start time: {start_time}')
                    duration = 20
                    output_audio_path = f'Processing/Audio/trimmed_audio_{video_index}.mp4'
                    trim_audio(input_audio_path, start_time, duration, output_audio_path)

                    input_video_path = f'Processing/Video/trimmed_video_{video_index - 1}.mp4'
                    input_audio_path = f'Processing/Audio/trimmed_audio_{video_index}.mp4'
                    output_video_path = f'Output/output_video_{video_count}.mp4'
                    add_audio_to_video(input_video_path, input_audio_path, output_video_path)
                    video_count += 1
                video_index += 1
                segment_start_frame = frame_count
                current_segment_frames = []

    # Check if there's a leftover segment at the end that should be saved
    if segment_start_frame is not None and len(current_segment_frames) >= frames_per_segment:
        out = cv2.VideoWriter(f'video{video_index}.mp4', fourcc, fps, (int(cap.get(3)), int(cap.get(4))))
        for f in current_segment_frames:
            out.write(f)
        out.release()

    cap.release()

split_input_video('Input/input_video.mp4')