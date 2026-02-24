#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
#import tf_transformations
from scipy.spatial.transform import Rotation as R


#Define color detection node
class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        #Both ROS2 nodes
        ## Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        ## Publisher
        self.coords_pub = self.create_publisher(String, '/color_coordinates', 10)

        # OpenCV bridge for converting ROS Image to OpenCV format
        self.bridge = CvBridge()

        # TF2 setup
        ## Create a TF buffer to store all known transforms and allow lookup of transforms between frames at different times
        self.tf_buffer = tf2_ros.Buffer() 
        ## Create a TF listener to fill the buffer with transforms pubslished by other nodes (like the robot state publisher)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera intrinsic parameters (from your SDF)
        ##focal points
        self.f = 381.36
        ##principal point (center of the image)
        self.cx = 320.0
        self.cy = 240.0

        self.get_logger().info("Color Detector Node Started with TF2 lookup transform")

    def image_callback(self, msg):
        try:
            # Convert ROS Image -> OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges (HSV)
        color_ranges = {
            "R": [(0, 120, 70), (10, 255, 255)],
            "G": [(55, 200, 200), (60, 255, 255)],
            "B": [(90, 200, 200), (128, 255, 255)]
        }

        for color_id, (lower, upper) in color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            #masks help computer isolate specific colors in the image by creating a binary image 
            #where the pixels that fall within the specified color range are set to white (255) and all other pixels are set to black (0)
            mask = cv2.inRange(hsv, lower, upper)

            # Noise removal
            ##erosion removes small white noise, while dilation increases the white region in the mask.
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Find contours
            ##find continous regions in the binary mask that represent detected objects of the specified color.
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


            for cnt in contours:
                ##filters out small contours that are likely noise by checking
                #if the area of the contour is greater than a certain threshold (in this case, 1 pixel).
                if cv2.contourArea(cnt) > 1:  # Increased minimum area threshold
                    #x, y are the top-left corner of the bounding box, w and h are the width and height of the bounding box
                    x, y, w, h = cv2.boundingRect(cnt)
                    #cx_pix and cy_pix represent the pixel coordinates of the center of the detected object in the image.
                    cx_pix, cy_pix = x + w // 2, y + h // 2

                    # Draw bounding box + label
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, color_id, (x, y - 10),#-10 is to give clearance between the letter and the box outline
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    # Convert pixel -> camera_optical frame
                    Y = 1  # Fixed height of the camera above the table (adjust as needed)
                    X = ((cx_pix - self.cx) * Y)/ self.f
                    Z = ((cy_pix - self.cy) * Y)/ self.f 
             
                    try:
                        # Lookup transform camera_link -> panda_link0
                        # Use Time(seconds=0) for latest available transform
                        # return translation + rotation (quaternion) 
                        t = self.tf_buffer.lookup_transform(
                            "panda_link0", 
                            "camera_link_optical", 
                            rclpy.time.Time(),
                            timeout=Duration(seconds=1.0))

                        # Convert to numpy transform matrix
                        trans = np.array([
                            t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z
                        ])
                        
                        rot = [
                            t.transform.rotation.x,
                            t.transform.rotation.y,
                            t.transform.rotation.z,
                            t.transform.rotation.w
                        ]
                        
                        # Create 4x4 transformation matrix
                        R_obj = R.from_quat(rot)
                        T = R_obj.as_matrix()
                        T = np.array(T)
                        
                        
                        
                        #T = tf_transformations.quaternion_matrix(rot)
                        col = np.array(trans.reshape(-1, 1))
                        T = np.hstack((T, col))

                        # Transform point from camera_optical frame to base frame
                        pt_cam = np.array([X, Y, Z, 1.0])
                        pt_base = T @ pt_cam

                        #when 2D pixel coordinates are transformed into 3D world coordinates, image is projected as 2D onto the table
                        #however the actual blocks sit on top of the table at a fixed height
                        #if this error is not accounted for, the robot will attempt to pick up the block at its intersection with the table
                        #using the known height of the blocks, we can manually correct this error
                        # Adjust Z coordinates
                        pt_base[2] += 0.31 

                        # Publish color ID + coordinates in panda_link0 frame
                        msg_str = f"{color_id},{pt_base[0]:.3f},{pt_base[1]:.3f},{pt_base[2]:.3f}"
                        self.coords_pub.publish(String(data=msg_str))
                        self.get_logger().info(msg_str)
                        
                    except (tf2_ros.LookupException, 
                            tf2_ros.ConnectivityException, 
                            tf2_ros.ExtrapolationException) as e:
                        self.get_logger().warn(f"TF lookup failed: {e}")
                    except Exception as e:
                        self.get_logger().error(f"Unexpected error in TF transform: {e}")

        # Show image in window
        try:
            cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Color Detection", 640, 320)
            cv2.imshow("Color Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"OpenCV display error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()