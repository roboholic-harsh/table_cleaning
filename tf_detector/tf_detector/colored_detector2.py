# dont use this anywhere else, this is just a backup of the old code before I refactored it to be more modular and cleaner. The new code is in colored_detector.py, this is just here for reference and backup purposes.

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from image_geometry import PinholeCameraModel

class ColouredTFDetector(Node):
    def __init__(self):
        super().__init__('coloured_tf_detector')
        
        # 1. Setup TF (The "GPS" for the camera)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 2. Camera Utils
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False
        
        # 3. Subscribers
        self.create_subscription(CameraInfo, '/camera/overhead_camera/camera_info', self.info_callback, 1)
        self.create_subscription(Image, '/camera/overhead_camera/image_raw', self.image_callback, 1)

        # 4. Publishers
        # Coordinates for the robot
        self.red_pub   = self.create_publisher(Point, '/red_block_coords', 10)
        self.green_pub = self.create_publisher(Point, '/green_block_coords', 10)
        self.blue_pub  = self.create_publisher(Point, '/blue_block_coords', 10)
        
        # Visual Debugging (For Rviz)
        # self.debug_img_pub = self.create_publisher(Image, '/detector/debug_image', 10)
        self.marker_pub = self.create_publisher(Marker, '/block_marker_tf', 10)

        # 5. Config (Table Height)
        # 1.0m (Table) + 0.04m (Block) = 1.04m target Z
        self.target_z = .12

        print("--- Multi-Color TF Detector Started ---")
        print("Displaying detections locally and publishing to /detector/debug_image")

    def info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True

    def image_callback(self, msg):
        if not self.camera_info_received:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # --- PROCESS COLORS SEPARATELY ---
        
        # 1. RED (Wrap-around hue)
        mask_r1 = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255]))
        mask_r2 = cv2.inRange(hsv, np.array([170, 50, 50]), np.array([180, 255, 255]))
        mask_red = mask_r1 + mask_r2
        self.process_color(cv_image, mask_red, (0, 0, 255), self.red_pub, "Red")

        # 2. GREEN
        mask_green = cv2.inRange(hsv, np.array([35, 50, 50]), np.array([85, 255, 255]))
        self.process_color(cv_image, mask_green, (0, 255, 0), self.green_pub, "Green")

        # 3. BLUE
        mask_blue = cv2.inRange(hsv, np.array([100, 50, 50]), np.array([140, 255, 255]))
        self.process_color(cv_image, mask_blue, (255, 0, 0), self.blue_pub, "Blue")

        # --- DISPLAY ---
        # A. Publish to Rviz
        # try:
        #     ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        #     self.debug_img_pub.publish(ros_image)
        # except Exception:
        #     pass

        # B. Show Local Window
        cv2.imshow("Sorting Cam", cv_image)
        cv2.waitKey(1)

    def process_color(self, img_to_draw, mask, viz_color_bgr, publisher, label_prefix):
        """
        Finds blobs, identifies Top-Right target, and Draws on image.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_objects = []

        # A. Collect World Coordinates
        for c in contours:
            if cv2.contourArea(c) > 300:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                
                world_coords = self.get_world_coords((cx, cy), rclpy.time.Time())
                
                if world_coords:
                    wx, wy, wz = world_coords
                    detected_objects.append({
                        'px': x, 'py': y, 'pw': w, 'ph': h,
                        'wx': wx, 'wy': wy, 'wz': wz
                    })

        # B. Find Target (Top Right)
        target_obj = None
        if detected_objects:
            # Metric: Maximize (X - Y)
            target_obj = max(detected_objects, key=lambda b: b['wx'] - b['wy'])
            
            # Publish
            p = Point()
            p.x, p.y, p.z = target_obj['wx'], target_obj['wy'], target_obj['wz']
            publisher.publish(p)
            
            # Rviz Marker
            self.publish_marker(p.x, p.y, p.z, viz_color_bgr)

        # C. Draw Visuals on Image
        for obj in detected_objects:
            x, y, w, h = obj['px'], obj['py'], obj['pw'], obj['ph']
            
            if obj == target_obj:
                # THICK Box + Coordinate Label
                cv2.rectangle(img_to_draw, (x, y), (x + w, y + h), viz_color_bgr, 3)
                label = f"{label_prefix} TGT ({obj['wx']:.2f}, {obj['wy']:.2f})"
            else:
                # Thin Box
                cv2.rectangle(img_to_draw, (x, y), (x + w, y + h), viz_color_bgr, 1)
                label = f"{label_prefix}"

            cv2.putText(img_to_draw, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, viz_color_bgr, 2)


    def get_world_coords(self, uv_pixel, timestamp):
        try:
            rect_pixel = self.camera_model.rectifyPoint(uv_pixel)
            ray_3d = self.camera_model.projectPixelTo3dRay(rect_pixel)
            
            ray_pt = PointStamped()
            ray_pt.header.frame_id = self.camera_model.tfFrame()
            ray_pt.point.x, ray_pt.point.y, ray_pt.point.z = ray_3d
            
            cam_pt = PointStamped()
            cam_pt.header.frame_id = self.camera_model.tfFrame()
            
            if not self.tf_buffer.can_transform("world", self.camera_model.tfFrame(), rclpy.time.Time()):
                return None

            ray_world = self.tf_buffer.transform(ray_pt, "world")
            cam_world = self.tf_buffer.transform(cam_pt, "world")

            ox, oy, oz = cam_world.point.x, cam_world.point.y, cam_world.point.z
            dx = ray_world.point.x - ox
            dy = ray_world.point.y - oy
            dz = ray_world.point.z - oz

            if abs(dz) < 0.001: return None

            t = (self.target_z - oz) / dz
            return (ox + t * dx, oy + t * dy, self.target_z)

        except Exception:
            return None

    def publish_marker(self, x, y, z, color_tuple):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.ns = f"marker_{color_tuple}" 
        marker.id = 0
        marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.b = color_tuple[0] / 255.0
        marker.color.g = color_tuple[1] / 255.0
        marker.color.r = color_tuple[2] / 255.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ColouredTFDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()