import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import threading
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            1
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        self.state = 'SEARCHING'
        self.last_seen_cx = None
        self.running = True
        self.last_line_seen_time = None
        self.line_loss_timeout = 0.5

        # Paraméterek
        self.search_angular_speed = 0.08
        self.align_linear_speed = 0.05
        self.align_angular_speed = 0.06
        self.track_linear_speed = 0.10
        self.track_angular_gain = 0.0003
        self.min_contour_area = 500

        self.align_counter = 0
        self.last_seen_direction = 0.0
        self.search_start_time = time.time()

        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    # ... (spin_thread_func, image_callback, display_image változatlan)

    def process_image(self, img):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        rows, cols = img.shape[:2]

        _, L, _ = self.convert2hls(img)
        L_masked, mask = self.apply_polygon_mask(L)
        lightnessMask = self.threshold_binary(L_masked, (180, 255))
        stackedMask = np.dstack((lightnessMask, lightnessMask, lightnessMask))
        contourMask = stackedMask.copy()
        crosshairMask = stackedMask.copy()

        contours, _ = cv2.findContours(lightnessMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lightnessMask = cv2.addWeighted(mask, 0.2, lightnessMask, 0.8, 0)

        valid_contour = False
        cx = cy = None

        if contours:
            valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_contour_area]
            if valid_contours:
                biggest_contour = max(valid_contours, key=cv2.contourArea)
                M = cv2.moments(biggest_contour)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self.last_seen_cx = cx
                    self.last_line_seen_time = time.time()
                    valid_contour = True
                    self.last_seen_direction = cols / 2 - cx  # irány elmentése

                    cv2.drawContours(contourMask, [biggest_contour], -1, (0, 255, 0), 10)
                    cv2.circle(contourMask, (cx, cy), 20, (0, 0, 255), -1)
                    cv2.line(crosshairMask, (cx, 0), (cx, rows), (0, 0, 255), 10)
                    cv2.line(crosshairMask, (0, cy), (cols, cy), (0, 0, 255), 10)
                    cv2.line(crosshairMask, (cols // 2, 0), (cols // 2, rows), (255, 0, 0), 10)

        if valid_contour:
            self.search_start_time = time.time()  # reset keresési idő
            if self.state == 'SEARCHING':
                self.state = 'ALIGNING'
                self.get_logger().info("Line detected, switching to ALIGNING")

            if self.state == 'ALIGNING':
                error_x = cols / 2 - cx
                error_y = rows - cy

                if abs(error_x) < 50 and cy > rows * 0.85:
                    self.align_counter += 1
                    if self.align_counter >= 5:
                        self.state = 'TRACKING'
                        self.align_counter = 0
                        self.get_logger().info("Aligned with line, switching to TRACKING")
                else:
                    self.align_counter = 0
                    msg.linear.x = self.align_linear_speed * (error_y / rows)
                    msg.angular.z = self.align_angular_speed * (error_x / (cols / 2))

                self.get_logger().info(f"ALIGNING: cx={cx}, error_x={error_x:.1f}, lin_x={msg.linear.x:.3f}, ang_z={msg.angular.z:.3f}")

            elif self.state == 'TRACKING':
                #error_x = cols / 2 - cx
                #msg.linear.x = self.track_linear_speed
                #msg.angular.z = -self.track_angular_gain * error_x
                

                #msg.angular.z = max(min(msg.angular.z, 0.3), -0.3)
                #self.get_logger().info(f"TRACKING: cx={cx}, error_x={error_x:.1f}, lin_x={msg.linear.x:.3f}, ang_z={msg.angular.z:.3f}")

            
                error_x = cols / 2 - cx
                
                # Gyorsabb haladás, ha közel van a vonal középvonalához
                max_speed = 0.4    # Még gyorsabb haladás, de ha túl gyors, csökkentheted
                min_speed = 0.15   # Az alap minimális sebesség
                max_error = cols / 4  # Maximum eltérés a középvonaltól (pl. 160 pixel, ha 640 széles a kép)
                
                # Az eltérés arányának kiszámítása: ha kicsi, gyorsabb, ha nagy, akkor lassabb
                error_ratio = min(abs(error_x) / max_error, 1.0)
                
                # Sebesség csökkentés, ha nagy az eltérés
                speed_boost = max(0, 1.0 - error_ratio)  # min speed = gyorsabb, ha közel van a vonalhoz
                
                msg.linear.x = min_speed + (max_speed - min_speed) * speed_boost
                
                # Az irányultság korrekciója: növeld a sebesség nagyobb eltéréseknél
                msg.angular.z = -self.track_angular_gain * error_x
                msg.angular.z = max(min(msg.angular.z, 0.4), -0.4)  # Nagyobb engedélyezett szögeltérés

                # Log: Debug info, gyorsabb visszajelzés
                self.get_logger().info(f"TRACKING: cx={cx}, error_x={error_x:.1f}, lin_x={msg.linear.x:.3f}, ang_z={msg.angular.z:.3f}")

        else:
            if self.state in ['ALIGNING', 'TRACKING']:
                if self.last_line_seen_time and (time.time() - self.last_line_seen_time) < self.line_loss_timeout:
                    msg.linear.x = self.track_linear_speed if self.state == 'TRACKING' else 0.0
                    msg.angular.z = 0.0
                    self.get_logger().info(f"{self.state}: Holding last command, time since line: {time.time() - self.last_line_seen_time:.2f}s")
                else:
                    self.state = 'SEARCHING'
                    self.search_start_time = time.time()
                    self.get_logger().info("Line lost, switching to SEARCHING")

    


            if self.state == 'SEARCHING':
                # ha túl régóta keresünk, váltunk irányt
                if time.time() - self.search_start_time > 3.0:
                    self.last_seen_direction *= -1
                    self.search_start_time = time.time()
                    self.get_logger().info("No line for 3s, switching search direction")

                # irány az utolsó ismert alapján
                if self.last_seen_direction != 0:
                    direction = -1 if self.last_seen_direction < 0 else 1
                    msg.angular.z = direction * self.search_angular_speed
                else:
                    msg.angular.z = -self.search_angular_speed

                msg.linear.x = 0.03  # kis előremozgás
                self.get_logger().info(f"SEARCHING: ang_z={msg.angular.z:.3f}, lin_x={msg.linear.x:.2f}")

        self.publisher.publish(msg)
        return L_masked, contourMask, crosshairMask


    def convert2hls(self, img):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        return hls[:, :, 0], hls[:, :, 1], hls[:, :, 2]

    def apply_polygon_mask(self, img):
        mask = np.zeros_like(img)
        ignore_mask_color = 255
        imshape = img.shape
        vertices = np.array([[
            (0, imshape[0]),
            (imshape[1] * 0.3, imshape[0] * 0.4),
            (imshape[1] * 0.7, imshape[0] * 0.4),
            (imshape[1], imshape[0])
        ]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image, mask

    def threshold_binary(self, img, thresh=(180, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1
        return binary * 255

    def add_small_pictures(self, img, small_images, size=(160, 120)):
        x_base_offset = 40
        y_base_offset = 10
        x_offset = x_base_offset
        y_offset = y_base_offset
        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))
            img[y_offset:y_offset + size[1], x_offset:x_offset + size[0]] = small
            x_offset += size[0] + x_base_offset
        return img

    def stop_robot(self):
        msg = Twist()
        self.publisher.publish(msg)

    def stop(self):
        self.running = False
        self.spin_thread.join()

def main(args=None):
    print("OpenCV version: %s" % cv2.__version__)
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        node.display_image()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
