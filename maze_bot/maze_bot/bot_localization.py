import cv2
import numpy as np
from .utilities import ret_smallest_obj

class bot_localizer():
    def __init__(self):
        self.is_bg_extracted = False
        self.bg_model = []
        self.maze_og = []

        #Transformation(Crop+Rotated) Variables
        self.orig_X = 0
        self.orig_Y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.transform_arr = []
        self.orig_rot = 0
        self.orig_mat = 0

    
    def ret_rois_boundinghull(rois_mask, cnts):
        maze_enclosure = np.zeros_like(rois_mask)
        if cnts:
            cnts_ = np.concatenate(cnts)
            cnts_ = np.array(cnts_)
            cv2.fillConvexPoly(maze_enclosure, cnts_, 255)
        cnts_largest = cv2.findContours(maze_enclosure, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        hull = cv2.convexHull(cnts_largest[0])
        cv2.drawContours(maze_enclosure, [hull], 0, 255)
        return hull

    def update_frameofrefrence_parameters(self,X,Y,W,H,rot_angle):
        self.orig_X = X; self.orig_Y = Y; self.orig_rows = H; self.orig_cols = W; self.orig_rot = rot_angle # 90 degree counterClockwise
        self.transform_arr = [X,Y,W,H]
        # Rotation Matrix
        self.rot_mat = np.array(
                                [
                                 [ np.cos(np.deg2rad(self.orig_rot)) , np.sin(np.deg2rad(self.orig_rot))],
                                 [-np.sin(np.deg2rad(self.orig_rot)) , np.cos(np.deg2rad(self.orig_rot))]
                                ]
                               )
        self.rot_mat_rev = np.array(
                                [
                                 [ np.cos(np.deg2rad(-self.orig_rot)) , np.sin(np.deg2rad(-self.orig_rot))],
                                 [-np.sin(np.deg2rad(-self.orig_rot)) , np.cos(np.deg2rad(-self.orig_rot))]
                                ]
                               )
    

    def extract_bg(self, frame):
        #Extracting the mask of all ROIs
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, None, 3)
        cnts = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        roi_mask  =np.zeros((frame.shape[0], frame.shape[1]) dtype=np.uint)
        for idx,_ in enumerate(cnts):
            cv2.drawContours(roi_mask, cnts, idx, 255, -1)

        #Removing the car from these ROIs
        min_cntr_idx = ret_smallest_obj(cnts)
        rois_noCar_mask = roi_mask.copy()
        if min_cntr_idx != -1:
            cv2.drawContours(rois_noCar_mask, cnts, min_cntr_idx, 0, -1)
            #Draw car mask
            car_mask = np.zeros_like(roi_mask)
            cv2.drawContours(car_mask, cnts, min_cntr_idx, 255, -1)
            cv2.drawContours(car_mask, cnts, min_cntr_idx, 255, 3)
            Notcar_mask = cv2.bitwise_not(car_mask)
            frame_car_remvd = cv2.bitwise_and(frame, frame, mask = Notcar_mask)
            
            base_clr = frame_car_remvd[0][0]
            ground_replica = np.ones_like(frame)*base_clr

            #Generating BG_model
            self.bg_model = cv2.bitwise_and(ground_replica, ground_replica, mask=car_mask)
            self.bg_model = cv2.bitwise_or(self.bg_model ,frame_car_remvd)

        #Extracting the maze (Frame of reference) maze entry on top
        #finding the dimensions of the hull enclosing the largest contour
        hull = self.ret_rois_boundinghull(roi_mask, cnts)
        [X,Y,W,H] = cv2.boundingRect(hull)

        #Cropping maze_mask from the image
        maze = rois_noCar_mask(Y:Y+H, X:X+W)
        
        maze_occupencygrid = cv2.bitwise_not(maze)
        self.maze_og = cv2.rotate(maze_occupencygrid, cv2.ROTATE_90_COUNTERCLOCKWISE)

        #storing crop and rot params required to maintain the frame of ref in the orig image
        self.update_frameofrefrence_parameters(X,Y,W,H,90)

        cv2.imshow('1a. rois mask', roi_mask)
        cv2.imshow('1b. frame_car_remvd', frame_car_remvd)
        cv2.imshow('1c. ground_replica', ground_replica)
        cv2.imshow('1d. bg_model', self.bg_model)
        cv2.imshow('2. maze_og', self.maze_og)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def localize_bot(self, curr_frame, frame_disp):
        if not self.is_bg_extracted:
            self.extract_bg(curr_frame)
            self.is_bg_extracted = True