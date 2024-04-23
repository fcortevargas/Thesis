import cv2
import subprocess
import numpy as np

def split_input_video(input_video_path):
    extract_audio_from_video(input_video_path, 'Input/input_audio.aac')
    cap = cv2.VideoCapture(input_video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    frames_per_segment = 20 * fps  # 20 seconds of frames
    process_video_segments(cap, fps, frames_per_segment)
    cap.release()

def extract_audio_from_video(input_video_path, output_audio_path):
    cmd = [
        'ffmpeg',
        '-i', input_video_path,
        '-vn', '-acodec', 'copy',
        output_audio_path
    ]
    subprocess.run(cmd)

def process_video_segments(cap, fps, frames_per_segment):
    """Processes video segments, trimming and adding audio as necessary."""
    segment_start_frame_index = None
    current_segment_frames = []
    video_index = 1

    while True:
        ret, frame = cap.read()
        
        if not ret:
            raise Exception("No magenta frames found!")

        frame_index = cap.get(cv2.CAP_PROP_POS_FRAMES)
        frame_size = (int(cap.get(3)), int(cap.get(4)))

        if is_magenta_present(frame):
            first_magenta_frame_index = frame_index
        
            while True:
                _, frame = cap.read()

                frame_index = cap.get(cv2.CAP_PROP_POS_FRAMES)

                if frame_index == first_magenta_frame_index + 2.5 * np.round(fps):
                    break

            segment_start_frame_index, current_segment_frames = update_segment(frame_index, current_segment_frames, frame, segment_start_frame_index)
            break

    is_magenta_segment = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_index = cap.get(cv2.CAP_PROP_POS_FRAMES)
        frame_size = (int(cap.get(3)), int(cap.get(4)))

        if not is_magenta_segment:
            if frame_index < segment_start_frame_index + 20 * np.round(fps):
                segment_start_frame_index, current_segment_frames = update_segment(frame_index, current_segment_frames, frame, segment_start_frame_index)
            elif frame_index == segment_start_frame_index + 20 * np.round(fps):
                handle_video_segment(fps, current_segment_frames, video_index, frame_index, frame_size)
                video_index = update_video_index(video_index)
                segment_start_frame_index, current_segment_frames = reset_segment()
                segment_start_frame_index, current_segment_frames = update_segment(frame_index, current_segment_frames, frame, segment_start_frame_index)
                is_magenta_segment = True
        elif frame_index == segment_start_frame_index + 2.5 * np.round(fps):
            is_magenta_segment = False
            segment_start_frame_index, current_segment_frames = reset_segment()
            segment_start_frame_index, current_segment_frames = update_segment(frame_index, current_segment_frames, frame, segment_start_frame_index)


def is_magenta_present(frame, threshold=59):
    """Check if there is a significant amount of magenta in the frame."""
    # Convert frame to RGB (OpenCV uses BGR by default)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    magenta_lower = np.array([255 - threshold, 0, 255 - threshold])
    magenta_upper = np.array([255, threshold, 255])
    mask = cv2.inRange(frame_rgb, magenta_lower, magenta_upper)
    return np.sum(mask) > 0

def should_save_segment(segment_start_frame_index, current_segment_frames, frames_per_segment):
    """Determines if the current video segment should be saved."""
    return segment_start_frame_index is not None and len(current_segment_frames) >= frames_per_segment

def handle_video_segment(fps, current_segment_frames, video_index, frame_index, frame_size):
    """Handles saving and processing of a video segment."""
    video_segment_path = f'Processing/Video/trimmed_video_{video_index}.mp4'
    save_video_segment(current_segment_frames, video_segment_path, fps, frame_size)
    if video_index % 2 == 0:
        process_audio_for_segment(frame_index, fps, video_index)

def save_video_segment(frames, output_path, fps, frame_size, fourcc=cv2.VideoWriter_fourcc(*'mp4v')):
    """Saves a list of frames as a video segment."""
    out = cv2.VideoWriter(output_path, fourcc, fps, frame_size)
    for f in frames:
        out.write(f)
    out.release()

def process_audio_for_segment(frame_index, fps, video_index):
    """Processes audio for the current video segment."""
    start_time = frame_index / fps - 20
    duration = 20
    input_audio_path = 'Input/input_audio.aac'
    output_audio_path = f'Processing/Audio/trimmed_audio_{video_index}.mp4'
    trim_audio(input_audio_path, start_time, duration, output_audio_path)
    combine_audio_video(video_index)

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

def combine_audio_video(video_index):
    """Combines audio and video for the final output."""
    input_video_path = f'Processing/Video/trimmed_video_{video_index - 1}.mp4'
    input_audio_path = f'Processing/Audio/trimmed_audio_{video_index}.mp4'
    output_video_path = f'Output/output_video_{video_index // 2}.mp4'
    add_audio_to_video(input_video_path, input_audio_path, output_video_path)

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

def update_video_index(video_index):
    """Updates index for video and audio processing."""
    return video_index + 1

def reset_segment():
    """Resets the current segment's start frame index and frames list."""
    return None, []

def update_segment(frame_index, current_segment_frames, frame, segment_start_frame_index):
    """Updates the current segment with new frames."""
    if segment_start_frame_index is None:
        segment_start_frame_index = frame_index
    current_segment_frames.append(frame)
    return segment_start_frame_index, current_segment_frames

def segment_ends(frame_index, segment_start_frame_index, frames_per_segment):
    """Checks if the current segment has reached its end."""
    return frame_index - segment_start_frame_index >= frames_per_segment

def start_new_segment(frame_index, current_segment_frames):
    """Starts a new segment after the previous one ends."""
    return frame_index, []

def finalize_last_segment(segment_start_frame_index, current_segment_frames, frames_per_segment, fps, frame_size, video_index):
    """Handles the last video segment if it needs to be saved."""
    if should_save_segment(segment_start_frame_index, current_segment_frames, frames_per_segment):
        video_segment_path = f'Processing/Video/trimmed_video_{video_index}.mp4'
        save_video_segment(current_segment_frames, video_segment_path, fps, frame_size)
        if video_index % 2 == 0:
            process_audio_for_segment(segment_start_frame_index * fps, fps, video_index)

# Call the function with the path to your input video]
split_input_video('Input/input_video.mp4')