#!/usr/bin/env python2
from __future__ import print_function

import sys
import rospy
import cv2


import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class obstacle_segmenter():



    def __init__(self):
        self.image_pub = rospy.Publisher("/robot_obstacle_seg_output",Image, queue_size = 1)

        self.image_wh = [640,360]
        self.movement_th = 30
        self.vertical_cropping = np.asarray([0.0,2.0,1.0,1.0,1.0,2.0]) * self.image_wh[0] * (1.0/7)
        self.horizontal_cropping = np.asarray([0.0,2.0,1.0]) * (self.image_wh[1] - self.movement_th) * (1.0/3)

        self.vertical_cropping = map(int,self.vertical_cropping)
        self.horizontal_cropping = map(int,self.horizontal_cropping)

        # define grid points
        self.x0 = self.vertical_cropping[0]                     #  y0,x0-----x1-----x2-----x3-----x4--------x5
        self.x1 = self.x0 + self.vertical_cropping[1]           #  |         |      |      |      |         |
        self.x2 = self.x1 + self.vertical_cropping[2]           #  |    A1   |  A2  |  A3  |  A4  |   A5    |
        self.x3 = self.x2 + self.vertical_cropping[3]           #  |         |      |      |      |         |
        self.x4 = self.x3 + self.vertical_cropping[4]           #  y1----------------------------------------
        self.x5 = self.x4 + self.vertical_cropping[5]           #  |    B1   |  B2  |  B3  |  B4  |   B5    |
        self.y0 = self.horizontal_cropping[0]                   #  |         |      |      |      |         |
        self.y1 = self.y0 + self.horizontal_cropping[1]         #  y2----------------------------------------
        self.y2 = self.y1 + self.horizontal_cropping[2]         #  |    C1   |  C2  |  C3  |  C4  |   C5    |
        self.y3 = self.image_wh[1]                              #  y3----------------------------------------


        self.occupancy_grid = [[0 for i in range(len(self.vertical_cropping))] for j in range(len(self.horizontal_cropping))]


        self.bridge = CvBridge()
        
    def get_grid_region(self, region):
 
        if region == "A1":  return [(self.x0, self.y0),(self.x1, self.y1)]
                
        elif region == "A2":  return [(self.x1, self.y0),(self.x2, self.y1)]

        elif region == "A3":  return [(self.x2, self.y0),(self.x3, self.y1)]
                 
        elif region == "A4":  return [(self.x3, self.y0),(self.x4, self.y1)]
                 
        elif region == "A5":  return [(self.x4, self.y0),(self.x5, self.y1)]
                
        elif region == "B1":  return [(self.x0, self.y1),(self.x1, self.y2)]
                 
        elif region == "B2":  return [(self.x1, self.y1),(self.x2, self.y2)]
                 
        elif region == "B3":  return [(self.x2, self.y1),(self.x3, self.y2)]
                 
        elif region == "B4":  return [(self.x3, self.y1),(self.x4, self.y2)]
                
        elif region == "B5":  return [(self.x4, self.y1),(self.x5, self.y2)]
                 
        elif region == "C1":  return [(self.x0, self.y2),(self.x1, self.y3)]
                 
        elif region == "C2":  return [(self.x1, self.y2),(self.x2, self.y3)]
                
        elif region == "C3":  return [(self.x2, self.y2),(self.x3, self.y3)]
                 
        elif region == "C4":  return [(self.x3, self.y2),(self.x4, self.y3)]
                
        elif region == "C5":  return [(self.x4, self.y2),(self.x5, self.y3)]
                 
        else: return None  
        
    def compute_fill_ratio(self, mask_grid, coords):
        crop = mask_grid[coords[0][1]:coords[1][1], coords[0][0]:coords[1][0]]
        mean_values = cv2.mean(crop)
        return mean_values[0]/255


    def get_obstacle_mask(self):
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
        mask_grid = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # compute the fill of crop regions
        coords_A1 = self.get_grid_region("A1")
        fill_ratio_A1 = self.compute_fill_ratio(mask_grid, coords_A1)
        
        coords_A2 = self.get_grid_region("A2")
        fill_ratio_A2 = self.compute_fill_ratio(mask_grid, coords_A2)
        
        coords_A3 = self.get_grid_region("A3")
        fill_ratio_A3 = self.compute_fill_ratio(mask_grid, coords_A3)
        
        coords_A4 = self.get_grid_region("A4")
        fill_ratio_A4 = self.compute_fill_ratio(mask_grid, coords_A4)
        
        coords_A5 = self.get_grid_region("A5")
        fill_ratio_A5 = self.compute_fill_ratio(mask_grid, coords_A5)
        
        coords_B1 = self.get_grid_region("B1")
        fill_ratio_B1 = self.compute_fill_ratio(mask_grid, coords_B1)
        
        coords_B2 = self.get_grid_region("B2")
        fill_ratio_B2 = self.compute_fill_ratio(mask_grid, coords_B2)
        
        coords_B3 = self.get_grid_region("B3")
        fill_ratio_B3 = self.compute_fill_ratio(mask_grid, coords_B3)
        
        coords_B4 = self.get_grid_region("B4")
        fill_ratio_B4 = self.compute_fill_ratio(mask_grid, coords_B4)
        
        coords_B5 = self.get_grid_region("B5")
        fill_ratio_B5 = self.compute_fill_ratio(mask_grid, coords_B5)
        
        coords_C1 = self.get_grid_region("C1")
        fill_ratio_C1 = self.compute_fill_ratio(mask_grid, coords_C1)
        
        coords_C2 = self.get_grid_region("C2")
        fill_ratio_C2 = self.compute_fill_ratio(mask_grid, coords_C2)
        
        coords_C3 = self.get_grid_region("C3")
        fill_ratio_C3 = self.compute_fill_ratio(mask_grid, coords_C3)
        
        coords_C4 = self.get_grid_region("C4")
        fill_ratio_C4 = self.compute_fill_ratio(mask_grid, coords_C4)
        
        coords_C5 = self.get_grid_region("C5")
        fill_ratio_C5 = self.compute_fill_ratio(mask_grid, coords_C5)
        
        print(fill_ratio_A1)
        print(fill_ratio_C1)
        
        # plot grid on the mask
        cv2.rectangle(mask_grid, coords_A1[0], coords_A1[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_A2[0], coords_A2[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_A3[0], coords_A3[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_A4[0], coords_A4[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_A5[0], coords_A5[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_B1[0], coords_B1[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_B2[0], coords_B2[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_B3[0], coords_B3[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_B4[0], coords_B4[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_B5[0], coords_B5[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_C1[0], coords_C1[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_C2[0], coords_C2[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_C3[0], coords_C3[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_C4[0], coords_C4[1], (255,0,0), 2)
        cv2.rectangle(mask_grid, coords_C5[0], coords_C5[1], (255,0,0), 2)
        
        # plot level warning (increasing with red intensity) on the mask
        mask_grid[coords_A1[0][1]:coords_A1[1][1], coords_A1[0][0]:coords_A1[1][0], :] = mask_grid[coords_A1[0][1]:coords_A1[1][1], coords_A1[0][0]:coords_A1[1][0], :]/2
        mask_grid[coords_A1[0][1]:coords_A1[1][1], coords_A1[0][0]:coords_A1[1][0], 2] = mask_grid[coords_A1[0][1]:coords_A1[1][1], coords_A1[0][0]:coords_A1[1][0], 2] + int(fill_ratio_A1*(255/2))
        
        mask_grid[coords_A2[0][1]:coords_A2[1][1], coords_A2[0][0]:coords_A2[1][0], :] = mask_grid[coords_A2[0][1]:coords_A2[1][1], coords_A2[0][0]:coords_A2[1][0], :]/2
        mask_grid[coords_A2[0][1]:coords_A2[1][1], coords_A2[0][0]:coords_A2[1][0], 2] = mask_grid[coords_A2[0][1]:coords_A2[1][1], coords_A2[0][0]:coords_A2[1][0], 2] + int(fill_ratio_A2*(255/2))

        mask_grid[coords_A3[0][1]:coords_A3[1][1], coords_A3[0][0]:coords_A3[1][0], :] = mask_grid[coords_A3[0][1]:coords_A3[1][1], coords_A3[0][0]:coords_A3[1][0], :]/2
        mask_grid[coords_A3[0][1]:coords_A3[1][1], coords_A3[0][0]:coords_A3[1][0], 2] = mask_grid[coords_A3[0][1]:coords_A3[1][1], coords_A3[0][0]:coords_A3[1][0], 2] + int(fill_ratio_A3*(255/2))        
        
        mask_grid[coords_A4[0][1]:coords_A4[1][1], coords_A4[0][0]:coords_A4[1][0], :] = mask_grid[coords_A4[0][1]:coords_A4[1][1], coords_A4[0][0]:coords_A4[1][0], :]/2
        mask_grid[coords_A4[0][1]:coords_A4[1][1], coords_A4[0][0]:coords_A4[1][0], 2] = mask_grid[coords_A4[0][1]:coords_A4[1][1], coords_A4[0][0]:coords_A4[1][0], 2] + int(fill_ratio_A4*(255/2))

        mask_grid[coords_A5[0][1]:coords_A5[1][1], coords_A5[0][0]:coords_A5[1][0], :] = mask_grid[coords_A5[0][1]:coords_A5[1][1], coords_A5[0][0]:coords_A5[1][0], :]/2
        mask_grid[coords_A5[0][1]:coords_A5[1][1], coords_A5[0][0]:coords_A5[1][0], 2] = mask_grid[coords_A5[0][1]:coords_A5[1][1], coords_A5[0][0]:coords_A5[1][0], 2] + int(fill_ratio_A5*(255/2))

        mask_grid[coords_B1[0][1]:coords_B1[1][1], coords_B1[0][0]:coords_B1[1][0], :] = mask_grid[coords_B1[0][1]:coords_B1[1][1], coords_B1[0][0]:coords_B1[1][0], :]/2
        mask_grid[coords_B1[0][1]:coords_B1[1][1], coords_B1[0][0]:coords_B1[1][0], 2] = mask_grid[coords_B1[0][1]:coords_B1[1][1], coords_B1[0][0]:coords_B1[1][0], 2] + int(fill_ratio_B1*(255/2))        
        
        mask_grid[coords_B2[0][1]:coords_B2[1][1], coords_B2[0][0]:coords_B2[1][0], :] = mask_grid[coords_B2[0][1]:coords_B2[1][1], coords_B2[0][0]:coords_B2[1][0], :]/2
        mask_grid[coords_B2[0][1]:coords_B2[1][1], coords_B2[0][0]:coords_B2[1][0], 2] = mask_grid[coords_B2[0][1]:coords_B2[1][1], coords_B2[0][0]:coords_B2[1][0], 2] + int(fill_ratio_B2*(255/2)) 
        
        mask_grid[coords_B3[0][1]:coords_B3[1][1], coords_B3[0][0]:coords_B3[1][0], :] = mask_grid[coords_B3[0][1]:coords_B3[1][1], coords_B3[0][0]:coords_B3[1][0], :]/2
        mask_grid[coords_B3[0][1]:coords_B3[1][1], coords_B3[0][0]:coords_B3[1][0], 2] = mask_grid[coords_B3[0][1]:coords_B3[1][1], coords_B3[0][0]:coords_B3[1][0], 2] + int(fill_ratio_B3*(255/2)) 
        
        mask_grid[coords_B4[0][1]:coords_B4[1][1], coords_B4[0][0]:coords_B4[1][0], :] = mask_grid[coords_B4[0][1]:coords_B4[1][1], coords_B4[0][0]:coords_B4[1][0], :]/2
        mask_grid[coords_B4[0][1]:coords_B4[1][1], coords_B4[0][0]:coords_B4[1][0], 2] = mask_grid[coords_B4[0][1]:coords_B4[1][1], coords_B4[0][0]:coords_B4[1][0], 2] + int(fill_ratio_B4*(255/2))        
        
        mask_grid[coords_B5[0][1]:coords_B5[1][1], coords_B5[0][0]:coords_B5[1][0], :] = mask_grid[coords_B5[0][1]:coords_B5[1][1], coords_B5[0][0]:coords_B5[1][0], :]/2
        mask_grid[coords_B5[0][1]:coords_B5[1][1], coords_B5[0][0]:coords_B5[1][0], 2] = mask_grid[coords_B5[0][1]:coords_B5[1][1], coords_B5[0][0]:coords_B5[1][0], 2] + int(fill_ratio_B5*(255/2)) 
        
        mask_grid[coords_C1[0][1]:coords_C1[1][1], coords_C1[0][0]:coords_C1[1][0], :] = mask_grid[coords_C1[0][1]:coords_C1[1][1], coords_C1[0][0]:coords_C1[1][0], :]/2
        mask_grid[coords_C1[0][1]:coords_C1[1][1], coords_C1[0][0]:coords_C1[1][0], 2] = mask_grid[coords_C1[0][1]:coords_C1[1][1], coords_C1[0][0]:coords_C1[1][0], 2] + int(fill_ratio_C1*(255/2)) 
              
        mask_grid[coords_C2[0][1]:coords_C2[1][1], coords_C2[0][0]:coords_C2[1][0], :] = mask_grid[coords_C2[0][1]:coords_C2[1][1], coords_C2[0][0]:coords_C2[1][0], :]/2
        mask_grid[coords_C2[0][1]:coords_C2[1][1], coords_C2[0][0]:coords_C2[1][0], 2] = mask_grid[coords_C2[0][1]:coords_C2[1][1], coords_C2[0][0]:coords_C2[1][0], 2] + int(fill_ratio_C2*(255/2))        
      
        mask_grid[coords_C3[0][1]:coords_C3[1][1], coords_C3[0][0]:coords_C3[1][0], :] = mask_grid[coords_C3[0][1]:coords_C3[1][1], coords_C3[0][0]:coords_C3[1][0], :]/2
        mask_grid[coords_C3[0][1]:coords_C3[1][1], coords_C3[0][0]:coords_C3[1][0], 2] = mask_grid[coords_C3[0][1]:coords_C3[1][1], coords_C3[0][0]:coords_C3[1][0], 2] + int(fill_ratio_C3*(255/2)) 
        
        mask_grid[coords_C4[0][1]:coords_C4[1][1], coords_C4[0][0]:coords_C4[1][0], :] = mask_grid[coords_C4[0][1]:coords_C4[1][1], coords_C4[0][0]:coords_C4[1][0], :]/2
        mask_grid[coords_C4[0][1]:coords_C4[1][1], coords_C4[0][0]:coords_C4[1][0], 2] = mask_grid[coords_C4[0][1]:coords_C4[1][1], coords_C4[0][0]:coords_C4[1][0], 2] + int(fill_ratio_C4*(255/2)) 
        
        mask_grid[coords_C5[0][1]:coords_C5[1][1], coords_C5[0][0]:coords_C5[1][0], :] = mask_grid[coords_C5[0][1]:coords_C5[1][1], coords_C5[0][0]:coords_C5[1][0], :]/2
        mask_grid[coords_C5[0][1]:coords_C5[1][1], coords_C5[0][0]:coords_C5[1][0], 2] = mask_grid[coords_C5[0][1]:coords_C5[1][1], coords_C5[0][0]:coords_C5[1][0], 2] + int(fill_ratio_C5*(255/2)) 
        
                          
                        
        
        #print(mask)
        


        ros_image = self.bridge.cv2_to_imgmsg(mask_grid)
        self.image_pub.publish(ros_image)
    


def main(args):
  rospy.init_node('obstacle_segmenter', anonymous=True)
  ic = obstacle_segmenter()
  while not rospy.is_shutdown():
    try:
      ic.get_obstacle_mask()
    except KeyboardInterrupt:
      print("Shutting down")
      break

if __name__ == '__main__':
    main(sys.argv)


