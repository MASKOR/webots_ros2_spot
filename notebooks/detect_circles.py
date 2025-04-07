import os
import numpy as np
import cv2

output_dir = "./dataset"
color_images = sorted(
    [
        os.path.join(output_dir, "color", f)
        for f in os.listdir(os.path.join(output_dir, "color"))
    ]
)
depth_images = sorted(
    [
        os.path.join(output_dir, "depth", f)
        for f in os.listdir(os.path.join(output_dir, "depth"))
    ]
)

depth_width = 480
depth_height = 270
color_width = 640
color_height = 480
scale_x = depth_width / color_width
scale_y = depth_height / color_height

offset_x = -20
offset_y = 0

hough_dp = 1.0
hough_min_dist = 20
hough_param1 = 80
hough_param2 = 50
min_radius = 20
max_radius = 50

circles_data = []

for color_path, depth_path in zip(color_images, depth_images):
    color_img = cv2.imread(color_path)
    depth_img = np.load(depth_path)

    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=hough_dp,
        minDist=hough_min_dist,
        param1=hough_param1,
        param2=hough_param2,
        minRadius=min_radius,
        maxRadius=max_radius,
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for x, y, r in circles:
            x_scaled = int(x * scale_x) + offset_x
            y_scaled = int(y * scale_y) + offset_y
            r_scaled = int(r * scale_x)

            if 0 <= y_scaled < depth_height and 0 <= x_scaled < depth_width:
                y_min = max(0, y_scaled - r_scaled)
                y_max = min(depth_height, y_scaled + r_scaled)
                x_min = max(0, x_scaled - r_scaled)
                x_max = min(depth_width, x_scaled + r_scaled)
                depth_roi = depth_img[y_min:y_max, x_min:x_max]

                if depth_roi.size > 0:
                    valid_depths = depth_roi[depth_roi > 0]
                    if valid_depths.size > 0:
                        avg_depth = np.mean(valid_depths)
                        distance_meters = avg_depth / 1000.0
                        circles_data.append([x, y, r, distance_meters])
                        print(
                            f"Circle at ({x}, {y}), radius {r}, distance: {distance_meters:.2f} meters"
                        )

np.save(f"{output_dir}/circles_data.npy", np.array(circles_data))
print(f"Processed {len(color_images)} images and saved circle data.")
