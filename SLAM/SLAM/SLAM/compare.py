
import matplotlib.pyplot as plt 
import numpy as np
import cv2

mp4 = r"C:\Users\jaros\source\repos\SLAM\SLAM\exp\HD720_SN35428395_12-11-46_160_2300.mp4"

# Funkcja do odczytu konkretnej klatki
def get_frame(video_path, frame_idx):
    cap = cv2.VideoCapture(video_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"{video_path}: {total_frames} frames total")
    
    if frame_idx >= total_frames:
        print(f"Warning: frame {frame_idx} > {total_frames}, using last")
        frame_idx = total_frames - 1
    
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ret, frame = cap.read()
    cap.release()
    if ret:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return None


frame1 = get_frame(mp4, 1)  
frame2 = get_frame(mp4, 23000) 
if frame1 is not None and frame2 is not None:
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    axs[0].imshow(frame1)
    axs[0].set_title('Wheel Problem - Frame 100')
    axs[0].axis('off')
    axs[1].imshow(frame2)
    axs[1].set_title('Crooked Path - Frame 100')
    axs[1].axis('off')
    plt.tight_layout()
    plt.show()
    

    diff = np.abs(frame1.astype(np.float32) - frame2.astype(np.float32))
    plt.figure(figsize=(10, 5))
    plt.imshow(np.mean(diff, axis=2), cmap='hot')
    plt.title('Pixel Difference')
    plt.axis('off')
    plt.colorbar()
    plt.show()
else:
    pass





p1 = np.array([257,481])
p2 = np.array([267, 543])
delta = p2 - p1
dist = np.linalg.norm(delta)
angle = np.degrees(np.arctan2(delta[1], delta[0]))
print(f"Δ = {delta}, ||Δ|| = {dist:.2f}px, θ = {angle:.1f}°")