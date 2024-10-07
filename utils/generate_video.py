import cv2
import os

def images_to_video(image_folder, output_video="task.mp4", fps=10):
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    images.sort()



    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    video.release()
if __name__ == "__main__":
    images_to_video("/home/jas0n/Desktop/hybrid_a/ParkWithUncertainty/output/waypoint_v1/task0/camera_video_purpose")