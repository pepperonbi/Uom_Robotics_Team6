import pyrealsense2 as rs
import numpy as np
import cv2

class ObjectDetection:
    def __init__(self):
        self.pipeline = rs.pipeline()  # Define pipeline, create a pipeline
        self.config = rs.config()  # Define config
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)  # Configure depth stream
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)  # Configure color stream

        self.pipe_profile = self.pipeline.start(self.config)  # Start streaming
        self.align_to = rs.stream.color  # align_to is the stream type planned to align with the depth frame
        self.align = rs.align(self.align_to)  # rs.align performs alignment of depth frames with other frames

        self.color_intrin = None
        self.depth_intrin = None
        self.img_color = None
        self.img_depth = None
        self.aligned_depth_frame = None

    ''' 
    Get aligned image frames and camera parameters
    '''
    def get_aligned_images(self):
        frames = self.pipeline.wait_for_frames() # Wait to get image frames, get the frame set of color and depth
        aligned_frames = self.align.process(frames) # Get aligned frames, align depth frame with color frame

        self.aligned_depth_frame = aligned_frames.get_depth_frame() # Get depth frame from aligned frames
        aligned_color_frame = aligned_frames.get_color_frame() # Get color frame from aligned frames

        self.color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics # Get camera intrinsic parameters
        self.depth_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics # Get depth parameters (used for converting pixel coordinates to camera coordinates)

        self.img_color = np.asanyarray(aligned_color_frame.get_data()) # RGB image
        self.img_depth = np.asanyarray(self.aligned_depth_frame.get_data()) # Depth image (default 16-bit)
        return self.color_intrin, self.depth_intrin, self.img_color, self.img_depth, self.aligned_depth_frame
    
    def get_object_center(self,color):

        # Convert color space from BGR to HSV
        hsv = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2HSV)
 
        # Set the HSV threshold for yellow
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Set the HSV threshold for red
        lower_red = np.array([0, 43, 46])
        upper_red = np.array([10, 255, 255])

        # Set the HSV threshold for blue
        # lower_blue = np.array([100,43,46])
        # upper_blue = np.array([124,255,255])
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([124, 255, 255])

        # Create a mask for the region
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Combine the masks
        if color == 'yellow':
            mask = mask_yellow
        elif color == 'red':
            mask = mask_red
        elif color == 'blue':
            mask = mask_blue

        # Apply a morphological transformation to the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find the contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the center of the largest contour
        if len(contours) != 0:
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            center = (cX, cY)
            return contours, center
        else:
            return None, None

    ''' 
    Get 3D coordinates of the point
    '''   
    def get_3d_camera_coordinate(self, depth_pixel):
        x = depth_pixel[0]
        y = depth_pixel[1]
        dis = round(self.aligned_depth_frame.get_distance(x, y),4) # Get the depth corresponding to this pixel
        camera_coordinate = rs.rs2_deproject_pixel_to_point(self.depth_intrin, depth_pixel, dis) # Get the 3D coordinates in the camera coordinate system
        camera_coordinate = [round(coord, 4) for coord in camera_coordinate] 
        return dis, camera_coordinate

    
    def main(self,color):

        ''' 
        Get aligned image frames and camera parameters
        '''
        self.get_aligned_images()
        contours, center = self.get_object_center(color)
        
        
        '''
        Get 3D coordinates of the center point
        '''
        dis = 0
        if contours and center is not None:
            depth_pixel = [center[0], center[1]]
            dis, camera_coordinate = self.get_3d_camera_coordinate(depth_pixel)
            if dis != 0:
                print('depth: ',dis)
                print('camera_coordinate: ',camera_coordinate)

                '''
                Display image and annotations
                '''
                #### Mark the random point and its coordinates in the image ####
                cv2.drawContours(self.img_color, contours, -1, (0, 255, 0), 3)
                cv2.circle(self.img_color, depth_pixel, 8, [255, 0, 255], thickness=-1)
                cv2.putText(self.img_color, "Depth:" + str(dis) + " m", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, [0, 0, 255])
                cv2.putText(self.img_color, "X:" + str(camera_coordinate[0]) + " m", (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2, [255, 0, 0])
                cv2.putText(self.img_color, "Y:" + str(camera_coordinate[1]) + " m", (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.2, [255, 0, 0])
                cv2.putText(self.img_color, "Z:" + str(camera_coordinate[2]) + " m", (80, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.2, [255, 0, 0])



            #### Display the screen ####
            cv2.imshow('RealSence', self.img_color)
        return camera_coordinate if dis != 0 else None


if __name__ == "__main__":
    obj = ObjectDetection()
    colors = ['yellow','blue','red']
    coordinates = {color: [] for color in colors}
    # for color in colors:
    color = 'yellow'
    print('Finding the {} object'.format(color))
    while True:
        coordinate = obj.main(color)
        if coordinate is not None:
            coordinates[color].append(coordinate)
            if len(coordinates[color]) == 400:
                #coordinates.pop(150)
                avg_coordinate = np.mean(coordinates[color], axis=0)
                print('Average coordinate: '.format(color), avg_coordinate)
                break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    obj.pipeline.stop()
    cv2.destroyAllWindows()