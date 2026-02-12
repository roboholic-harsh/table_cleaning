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

class ProfessionalTopRightDetector(Node):
    def __init__(self):
        super().__init__('tf_top_right_detector')
        
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
        self.marker_pub = self.create_publisher(Marker, '/block_marker_tf', 10)
        self.coord_pub = self.create_publisher(Point, '/block_coordinates_tf', 10)

        # 5. Config
        self.target_z = 0.12  # Known table/block height
        
        print("--- TF Target Selector Started ---")
        print("Highlighting ALL blocks (Green). Targeting Top-Right (Blue).")

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

        # 1. Standard Color Detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Using the wide Red range to be safe
        mask1 = cv2.inRange(hsv, np.array([0, 100, 220]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 100, 220]), np.array([180, 255, 255]))
        mask3 = cv2.inRange(hsv, np.array([35, 100, 220]), np.array([85, 255, 255]))
        mask4 = cv2.inRange(hsv, np.array([100, 100, 220]), np.array([140, 255, 255]))
        mask = mask1 + mask2 + mask3 + mask4
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_objects = []

        # 2. Process EVERY block found
        for c in contours:
            if cv2.contourArea(c) > 500:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                
                # MAGIC STEP: Calculate 3D World Coordinates for THIS block immediately
                world_coords = self.get_world_coords((cx, cy), msg.header.stamp)
                
                if world_coords:
                    wx, wy, wz = world_coords
                    detected_objects.append({
                        'px': x, 'py': y, 'pw': w, 'ph': h, # Pixel info for drawing
                        'wx': wx, 'wy': wy, 'wz': wz        # World info for logic
                    })

        # 3. Select the "Top Right" Target
        target_obj = None
        if detected_objects:
            # Logic: Top Right = Max X (Far), Min Y (Right)
            # Score = X - Y. The higher the score, the more "Top-Right" it is.
            target_obj = max(detected_objects, key=lambda b: b['wx'] - b['wy'])
            
            # Publish Target
            p = Point()
            p.x, p.y, p.z = target_obj['wx'], target_obj['wy'], target_obj['wz']
            self.coord_pub.publish(p)
            self.publish_marker(p.x, p.y, p.z)

        # 4. Draw Visuals
        for obj in detected_objects:
            x, y, w, h = obj['px'], obj['py'], obj['pw'], obj['ph']
            
            if obj == target_obj:
                # BLUE for Target
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 3)
                label = f"TARGET: {obj['wx']:.2f}, {obj['wy']:.2f}"
                color = (255, 0, 0)
            else:
                # GREEN for others
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                label = f"{obj['wx']:.2f}, {obj['wy']:.2f}"
                color = (0, 255, 0)

            cv2.putText(cv_image, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        cv2.imshow("cube detector", cv_image)
        cv2.waitKey(1)

    def get_world_coords(self, uv_pixel, timestamp):
        """
        Converts 2D Pixel -> 3D World Coordinate using TF Ray Casting
        Returns (x, y, z) or None if transform fails.
        """
        try:
            # A. Pixel -> 3D Ray (Camera Frame)
            rect_pixel = self.camera_model.rectifyPoint(uv_pixel)
            ray_3d = self.camera_model.projectPixelTo3dRay(rect_pixel)
            
            # B. Create Ray Vector Stamped
            ray_pt = PointStamped()
            ray_pt.header.frame_id = self.camera_model.tfFrame()
            ray_pt.header.stamp = timestamp
            ray_pt.point.x, ray_pt.point.y, ray_pt.point.z = ray_3d
            
            # C. Create Camera Origin Stamped (0,0,0)
            cam_pt = PointStamped()
            cam_pt.header.frame_id = self.camera_model.tfFrame()
            cam_pt.header.stamp = timestamp
            
            # D. Transform both to World
            # Timeout is critical to sync with TF tree
            if not self.tf_buffer.can_transform("world", self.camera_model.tfFrame(), timestamp, timeout=Duration(seconds=0.5)):
                return None

            ray_world = self.tf_buffer.transform(ray_pt, "world")
            cam_world = self.tf_buffer.transform(cam_pt, "world")

            # E. Ray-Plane Intersection (Math)
            # Origin (ox, oy, oz) and Direction Vector (dx, dy, dz)
            ox, oy, oz = cam_world.point.x, cam_world.point.y, cam_world.point.z
            dx = ray_world.point.x - ox
            dy = ray_world.point.y - oy
            dz = ray_world.point.z - oz

            if abs(dz) < 0.001: return None # Parallel to plane check

            # Find 't' where Z = target_z
            t = (self.target_z - oz) / dz
            
            fx = ox + t * dx
            fy = oy + t * dy
            fz = self.target_z
            
            return (fx, fy, fz)

        except Exception as e:
            self.get_logger().warn(f"TF Math Error: {e}")
            return None

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05
        marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0 # BLUE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ProfessionalTopRightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()