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

def save_video_segment(frames, output_path, fps, frame_size, fourcc=cv2.VideoWriter_fourcc(*'mp4v')):
    """Saves a list of frames as a video segment."""
    out = cv2.VideoWriter(output_path, fourcc, fps, frame_size)
    for f in frames:
        out.write(f)
    out.release()

def process_video_segments(cap, fps, frames_per_segment):
    """Processes video segments, trimming and adding audio as necessary."""
    segment_start_frame = None
    current_segment_frames = []
    video_index = 1
    video_count = 1

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count = cap.get(cv2.CAP_PROP_POS_FRAMES)
        frame_size = (int(cap.get(3)), int(cap.get(4)))

        if is_magenta_present(frame):
            if should_save_segment(segment_start_frame, current_segment_frames, frames_per_segment):
                handle_video_segment(fps, current_segment_frames, video_index, frame_count, frame_size)
                video_index, video_count = update_indexes(video_index, video_count)
            segment_start_frame, current_segment_frames = reset_segment()
        else:
            segment_start_frame, current_segment_frames = update_segment(frame_count, current_segment_frames, frame, segment_start_frame)

            if segment_ends(frame_count, segment_start_frame, frames_per_segment):
                handle_video_segment(fps, current_segment_frames, video_index, frame_count, frame_size)
                segment_start_frame, current_segment_frames = start_new_segment(frame_count, current_segment_frames)
                video_index, video_count = update_indexes(video_index, video_count)

    finalize_last_segment(segment_start_frame, current_segment_frames, frames_per_segment, fps, frame_size, video_index)

def should_save_segment(segment_start_frame, current_segment_frames, frames_per_segment):
    """Determines if the current video segment should be saved."""
    return segment_start_frame is not None and len(current_segment_frames) >= frames_per_segment

def handle_video_segment(fps, current_segment_frames, video_index, frame_count, frame_size):
    """Handles saving and processing of a video segment."""
    video_segment_path = f'Processing/Video/trimmed_video_{video_index}.mp4'
    save_video_segment(current_segment_frames, video_segment_path, fps, frame_size)
    if video_index % 2 == 0:
        process_audio_for_segment(frame_count, fps, video_index)

def update_indexes(video_index, video_count):
    """Updates indexes for video and audio processing."""
    return video_index + 1, video_count + 1 if video_index % 2 == 0 else video_count

def reset_segment():
    """Resets the current segment's start frame and frames list."""
    return None, []

def update_segment(frame_count, current_segment_frames, frame, segment_start_frame):
    """Updates the current segment with new frames."""
    if segment_start_frame is None:
        segment_start_frame = frame_count
    current_segment_frames.append(frame)
    return segment_start_frame, current_segment_frames

def segment_ends(frame_count, segment_start_frame, frames_per_segment):
    """Checks if the current segment has reached its end."""
    return frame_count - segment_start_frame >= frames_per_segment

def start_new_segment(frame_count, current_segment_frames):
    """Starts a new segment after the previous one ends."""
    return frame_count, []

def process_audio_for_segment(frame_count, fps, video_index):
    """Processes audio for the current video segment."""
    start_time = frame_count / fps - 20
    duration = 20
    input_audio_path = 'Input/input_audio.aac'
    output_audio_path = f'Processing/Audio/trimmed_audio_{video_index}.mp4'
    trim_audio(input_audio_path, start_time, duration, output_audio_path)
    combine_audio_video(video_index)

def combine_audio_video(video_index):
    """Combines audio and video for the final output."""
    input_video_path = f'Processing/Video/trimmed_video_{video_index - 1}.mp4'
    input_audio_path = f'Processing/Audio/trimmed_audio_{video_index}.mp4'
    output_video_path = f'Output/output_video_{video_index // 2}.mp4'
    add_audio_to_video(input_video_path, input_audio_path, output_video_path)

def finalize_last_segment(segment_start_frame, current_segment_frames, frames_per_segment, fps, frame_size, video_index):
    """Handles the last video segment if it needs to be saved."""
    if should_save_segment(segment_start_frame, current_segment_frames, frames_per_segment):
        video_segment_path = f'Processing/Video/trimmed_video_{video_index}.mp4'
        save_video_segment(current_segment_frames, video_segment_path, fps, frame_size)
        if video_index % 2 == 0:
            process_audio_for_segment(segment_start_frame * fps, fps, video_index)

def split_input_video(input_video_path):
    extract_audio_from_video(input_video_path, 'Input/input_audio.aac')
    cap = cv2.VideoCapture(input_video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    frames_per_segment = 20 * fps  # 20 seconds of frames
    process_video_segments(cap, fps, frames_per_segment)
    cap.release()

# Call the function with the path to your input video]
split_input_video('Input/input_video.mp4')