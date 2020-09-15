#!/usr/bin/env python2
from __future__ import print_function

import sys
import rospy
import cv2


import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
from cv_bridge import CvBridge, CvBridgeError



class obstacle_segmenter():



    def __init__(self):
        print("Segmenter Initialization...",end="")
        self.image_pub = rospy.Publisher("/robot_obstacle_seg_output",Image, queue_size = 1)
        self.visual_range_grid_pub = rospy.Publisher("/visual_range_grid",Float32MultiArray, queue_size = 1)
        self.image_wh = [640,360]

        #pyramidal grid view to approximate laser scan representation
        #define grid setting:
        
        #-------- front region params ---------#
        self.h_trapezium = 0.4 * self.image_wh[1]    
        self.major_base_trapezium = self.image_wh[0]
        self.minor_base_trapezium = 0.3 * self.image_wh[0]   
        
        # ------- lateral regions params ---------#
        self.h1_lateral = 0.7 * self.image_wh[1]
        self.h2_depth = 0.15 * self.image_wh[1]

        #region labels
        self.regions = ["F", "FL", "FR"]

        self.front_left_mask = np.zeros([self.image_wh[1],self.image_wh[0]],dtype=np.uint8)
        self.front_mask = np.zeros([self.image_wh[1],self.image_wh[0]],dtype=np.uint8)
        self.front_right_mask = np.zeros([self.image_wh[1],self.image_wh[0]],dtype=np.uint8)

        #computing masks
        #       D_______C
        #       /       \
        #     A/_________\B
         
        #front points
        self.point_A = [0, self.image_wh[1]-1]
        self.point_B = [self.image_wh[0]-1, self.image_wh[1]-1]
        self.point_at_center_of_minor_base = [self.point_B[0]/2, self.image_wh[1]-1 - self.h_trapezium]
        self.point_C = [self.point_at_center_of_minor_base[0] + self.minor_base_trapezium/2, self.point_at_center_of_minor_base[1]]
        self.point_D = [self.point_at_center_of_minor_base[0] - self.minor_base_trapezium/2, self.point_at_center_of_minor_base[1]]
        
        #front-left points
        self.point_lateral_left = [self.point_A[0], self.image_wh[1]-1 - self.h1_lateral]
        self.point_depth_left = [self.point_D[0], self.point_D[1] - self.h2_depth]
        
        #front-right points
        self.point_lateral_right = [self.point_B[0], self.image_wh[1]-1 - self.h1_lateral]
        self.point_depth_right = [self.point_C[0], self.point_C[1] - self.h2_depth]
        
        # to plot image contours
        #point_A = (0, 0)
        #point_B = (self.image_wh[0]-1, 0)
        #point_C = (self.image_wh[0]-1, self.image_wh[1]-1)
        #point_D = (0, self.image_wh[1]-1)
        
        cv2.fillPoly(self.front_mask, np.array([[self.point_A, self.point_B, self.point_C, self.point_D]], dtype=np.int32), 255)
        cv2.fillPoly(self.front_left_mask, np.array([[self.point_A, self.point_D, self.point_depth_left, self.point_lateral_left]], dtype=np.int32), 255)
        cv2.fillPoly(self.front_right_mask, np.array([[self.point_B, self.point_C, self.point_depth_right, self.point_lateral_right]], dtype=np.int32), 255)


        # self.visual_range_grid = [[0 for i in range(len(self.vertical_cropping) - 1)] for j in range(len(self.horizontal_cropping))]
        self.visual_range_grid = np.zeros(3)
        self.image = None


        # for i in ["A","B","C"]:
        #   for j in range(1,6):
        #     self.regions.append(i+str(j))

        self.bridge = CvBridge()
        print("Done!")
        
    def get_grid_region(self, region):
 
        if region == "F":  return self.front_mask
                
        elif region == "FL":  return self.front_left_mask

        elif region == "FR":  return self.front_right_mask
                 
        else: return None  
        
    def compute_fill_ratio(self, mask_grid, region_mask):
        mask_grid_tmp = mask_grid[:,:,0]/255.0
        region_mask_tmp = region_mask/255.0
        mul_matrix = np.multiply(mask_grid_tmp,region_mask_tmp)
        sum_elem_in_mul_matrix = np.sum(mul_matrix)
        sum_elem_in_region_mask = np.sum(region_mask_tmp)
        return sum_elem_in_mul_matrix/sum_elem_in_region_mask

    def get_obstacle_mask(self,verbose = False):
        try:
          data = rospy.wait_for_message("/robot1/camera1/image_raw",Image)

          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
          
        # converting from BGR to HSV color space
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

        # Range for lower red
        lower_red = np.array([0,120,70])
        upper_red = np.array([10,255,255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        # Range for upper range
        lower_red = np.array([170,120,70])
        upper_red = np.array([180,255,255])
        mask2 = cv2.inRange(hsv,lower_red,upper_red)

        # Generating the final mask to detect red color
        mask = mask1+mask2
        #_, threshed_img = cv2.threshold(mask, 220, 255, cv2.THRESH_BINARY)
        #image, contours, hier = cv2.findContours(threshed_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        #for cnt in contours:
            # get convex hull
            #hull = cv2.convexHull(cnt)
            #cv2.fillPoly(mask, pts =[hull], color=255)  
        mask_grid = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        
        # compute the fill of crop regions
        for idx, region in enumerate(self.regions):
            region_mask = self.get_grid_region(region)
            fill_ratio = self.compute_fill_ratio(mask_grid, region_mask)
            self.visual_range_grid[idx] = fill_ratio
            #self.visual_range_grid[int(ord(region[0]) - ord("A"))][int(region[1]) - 1] =  fill_ratio

            # plot grid on the mas
              
              # plot level warning (increasing with red intensity) on the mask
              #mask_grid[coords[0][1]:coords[1][1], coords[0][0]:coords[1][0], :] = mask_grid[coords[0][1]:coords[1][1], coords[0][0]:coords[1][0], :]/2
              #mask_grid[coords[0][1]:coords[1][1], coords[0][0]:coords[1][0], 2] = mask_grid[coords[0][1]:coords[1][1], coords[0][0]:coords[1][0], 2] + int(fill_ratio*(255/2))

        #print(mask)
        msg = Float32MultiArray()

        # This is almost always zero there is no empty padding at the start of your data
        msg.layout.data_offset = 0 

        # create two dimensions in the dim array
        msg.layout.dim = [MultiArrayDimension()]
        # dim[0] is the vertical dimension of your matrix
        msg.layout.dim[0].label = "region"
        msg.layout.dim[0].size = 3
        msg.layout.dim[0].stride = 1

        msg.data = np.array(self.visual_range_grid)

        self.visual_range_grid_pub.publish(msg)

        #mask_grid = cv2.cvtColor(mask_grid, cv2.COLOR_GRAY2BGR)

        #cv2.line(cv_image, (self.x0, self.y1), (self.x5, self.y1), (255, 255, 0), 3) 
        #cv2.line(cv_image, (self.x0, self.y2), (self.x5, self.y2), (255, 255, 0), 3) 

        #cv2.line(cv_image, (self.x1, self.y0), (self.x1, self.y3), (0, 0, 255), 3) 
        #cv2.line(cv_image, (self.x2, self.y0), (self.x2, self.y3), (0, 0, 255), 3) 
        #cv2.line(cv_image, (self.x3, self.y0), (self.x3, self.y3), (255, 0, 255), 3) 
        #cv2.line(cv_image, (self.x4, self.y0), (self.x4, self.y3), (255, 0, 255), 3) 
        
        if verbose:
              pts_to_draw_front = np.array([self.point_A, self.point_B, self.point_C, self.point_D], np.int32)
              pts_to_draw_front = pts_to_draw_front.reshape((-1,1,2))
              pts_to_draw_front_left = np.array([self.point_A, self.point_D, self.point_depth_left, self.point_lateral_left], np.int32)
              pts_to_draw_front_left = pts_to_draw_front_left.reshape((-1,1,2))
              pts_to_draw_front_right = np.array([self.point_B, self.point_C, self.point_depth_right, self.point_lateral_right], np.int32)
              pts_to_draw_front_right = pts_to_draw_front_right.reshape((-1,1,2))
              cv2.polylines(mask_grid, [pts_to_draw_front, pts_to_draw_front_left, pts_to_draw_front_right], True, (255,0,255), 2)

        ros_image = self.bridge.cv2_to_imgmsg(mask_grid)
        self.image_pub.publish(ros_image)
        

def main(args):
  rospy.init_node('obstacle_segmenter', anonymous=True)
  ic = obstacle_segmenter()
  while not rospy.is_shutdown():
    try:
      ic.get_obstacle_mask(verbose=True)
    except KeyboardInterrupt:
      print("Shutting down")
      break

if __name__ == '__main__':
    main(sys.argv)