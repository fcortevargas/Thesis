import cv2
import numpy as np

def is_magenta_present(frame, threshold=50):
    """Check if there is a significant amount of magenta in the frame."""
    # Convert frame to RGB (OpenCV uses BGR by default)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    magenta_lower = np.array([255 - threshold, 0, 255 - threshold])
    magenta_upper = np.array([255, threshold, 255])
    mask = cv2.inRange(frame_rgb, magenta_lower, magenta_upper)
    return np.sum(mask) > 0

def process_video(video_path):
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    frames_per_segment = 20 * fps  # 20 seconds of frames
    segment_start_frame = None
    current_segment_frames = []
    video_number = 1
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # For MP4 output

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_count = cap.get(cv2.CAP_PROP_POS_FRAMES)

        if is_magenta_present(frame):
            # If magenta is present, check if the current segment should be saved
            if segment_start_frame is not None and len(current_segment_frames) >= frames_per_segment:
                out = cv2.VideoWriter(f'Output/output_video_{video_number}.mp4', fourcc, fps, (int(cap.get(3)), int(cap.get(4))))
                for f in current_segment_frames:
                    out.write(f)
                out.release()
                video_number += 1
            # Reset for the next segment
            segment_start_frame = None
            current_segment_frames = []
        else:
            if segment_start_frame is None:
                segment_start_frame = frame_count
            current_segment_frames.append(frame)

            # Check if we've reached the end of a 20-second segment
            if frame_count - segment_start_frame >= frames_per_segment:
                out = cv2.VideoWriter(f'Output/output_video_{video_number}.mp4', fourcc, fps, (int(cap.get(3)), int(cap.get(4))))
                for f in current_segment_frames:
                    out.write(f)
                out.release()
                video_number += 1
                segment_start_frame = frame_count
                current_segment_frames = []

    # Check if there's a leftover segment at the end that should be saved
    if segment_start_frame is not None and len(current_segment_frames) >= frames_per_segment:
        out = cv2.VideoWriter(f'video{video_number}.mp4', fourcc, fps, (int(cap.get(3)), int(cap.get(4))))
        for f in current_segment_frames:
            out.write(f)
        out.release()

    cap.release()

# Replace 'your_video.mp4' with the path to your video
process_video('Input/input_video.mp4')