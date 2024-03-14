import cv2
import numpy as np
import pyrealsense2 as rs
import queue

# def check_vector(model_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs):
#     # use the rotation and translation vectors to project the 3D model points to the 2D image plane
#     projected_points, _ = cv2.projectPoints(model_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

#     # reshape the projected points to a 2D array
#     projected_points = projected_points.astype(int).reshape(-1, 2)

#     # calculate the distance between the projected points and the image points
#     distances = np.sqrt(np.sum((image_points - projected_points)**2, axis=1))

#     # check if the distance is less than a threshold
#     threshold = 5
#     if np.all(distances < threshold):
#         print("The rotation and translation vectors are accurate.")
#     else:
#         print("The rotation and translation vectors are not accurate.")

class PoseEstimation:
    def __init__(self):
        # Set the leangth of the queue
        self.queue_length = 50
        self.rotation_queue = queue.Queue(maxsize=self.queue_length)
        self.translation_queue = queue.Queue(maxsize=self.queue_length)

        # set the output frequency
        self.output_frequency = 25
        self.counter = 0

        # 3D model points.
        self.model_points = np.array([
                                (0.0, 0.0, 0.0),             # Top left corner
                                (1.0, 0.0, 0.0),             # Top right corner
                                (1.0, 1.0, 0.0),             # Bottom right corner
                                (0.0, 1.0, 0.0)              # Bottom left corner
                            ])

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        # blue_lower = np.array([110, 50, 50])
        # blue_upper = np.array([130, 255, 255])

    def get_pose(self):
        rotation_average = None
        translation_average = None

        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        blurred = cv2.GaussianBlur(color_image, (11, 11), 0)
        median = cv2.medianBlur(blurred, 5)

        hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        mask= cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        # mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
        # mask = cv2.bitwise_or(mask_yellow, mask_blue)
        
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            center_x = int(rect[0][0])
            center_y = int(rect[0][1])

            depth = depth_frame.get_distance(center_x, center_y)

            # 2D image points. 
            image_points = np.array([
                                        (box[0][0], box[0][1]),     # Top left corner
                                        (box[1][0], box[1][1]),     # Top right corner
                                        (box[2][0], box[2][1]),     # Bottom right corner
                                        (box[3][0], box[3][1])      # Bottom left corner
                                    ], dtype="double")

            # Camera internals
            focal_length = color_image.shape[1]
            center = (color_image.shape[1]/2, color_image.shape[0]/2)
            camera_matrix = np.array(
                                [[focal_length, 0, center[0]],
                                [0, focal_length, center[1]],
                                [0, 0, 1]], dtype = "double"
                                )

            dist_coeffs = np.zeros((4,1))
            (success, rotation_vector, translation_vector) = cv2.solvePnP(self.model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

            # put the rotation and translation vectors into the queue
            if self.rotation_queue.full():
                self.rotation_queue.get()
            self.rotation_queue.put(rotation_vector)
            if self.translation_queue.full():
                self.translation_queue.get()
            self.translation_queue.put(translation_vector)

            # calculate the average of the rotation and translation vectors
            rotation_sum = np.zeros((3, 1))
            translation_sum = np.zeros((3, 1))
            for i in range(self.rotation_queue.qsize()):
                rotation_sum += self.rotation_queue.queue[i]
                translation_sum += self.translation_queue.queue[i]
            rotation_average = rotation_sum / self.rotation_queue.qsize()
            translation_average = translation_sum / self.translation_queue.qsize()
            
            # print the rotation and translation vectors
            self.counter += 1
            if self.counter == self.output_frequency:
                print("Rotation Vector:\n {0}".format(rotation_average))
                print("Translation Vector:\n {0}".format(rotation_average))
                #check_vector(model_points, rotation_average, translation_sum / translation_queue.qsize(), camera_matrix, dist_coeffs)
                self.counter = 0

            # Draw the box and the depth value
            cv2.drawContours(color_image, [box], 0, (0, 255, 0), 2)
            cv2.putText(color_image, f'Depth:{depth:.2f} m', (box[1][0], box[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        cv2.imshow('Color', color_image)
        cv2.imshow('Mask', mask)
        return rotation_average, translation_average


if __name__ == '__main__':
    pose = PoseEstimation()
    while True:
        rotation, translation = pose.get_pose()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()
    pose.pipeline.stop()
    