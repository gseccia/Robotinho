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
        print("Sgmenter Initialization...",end="")
        self.image_pub = rospy.Publisher("/robot_obstacle_seg_output",Image, queue_size = 1)
        self.occupancy_grid_pub = rospy.Publisher("/occupancy_grid",Float32MultiArray, queue_size = 1)
        
        self.image_wh = [640,360]
        self.movement_th = 30
        self.vertical_cropping = np.asarray([0.0,1.0,2.0,2.0,2.0,1.0]) * self.image_wh[0] * (1.0/8)
        self.horizontal_cropping = np.asarray([0.0,2.0,1.0]) * (self.image_wh[1] - self.movement_th) * (1.0/7)

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


        self.occupancy_grid = [[0 for i in range(len(self.vertical_cropping) - 1)] for j in range(len(self.horizontal_cropping))]
        self.image = None

        self.regions = []

        for i in ["A","B","C"]:
          for j in range(1,6):
            self.regions.append(i+str(j))

        self.bridge = CvBridge()
        print("Done!")
        
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
        mask_grid = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # compute the fill of crop regions
        for region in self.regions:
            coords = self.get_grid_region(region)
            fill_ratio = self.compute_fill_ratio(mask_grid, coords)
            self.occupancy_grid[int(ord(region[0]) - ord("A"))][int(region[1]) - 1] =  fill_ratio

            # plot grid on the mask
            if verbose:
              cv2.rectangle(mask_grid, coords[0], coords[1], (255,0,0), 2)
              
              # plot level warning (increasing with red intensity) on the mask
              mask_grid[coords[0][1]:coords[1][1], coords[0][0]:coords[1][0], :] = mask_grid[coords[0][1]:coords[1][1], coords[0][0]:coords[1][0], :]/2
              mask_grid[coords[0][1]:coords[1][1], coords[0][0]:coords[1][0], 2] = mask_grid[coords[0][1]:coords[1][1], coords[0][0]:coords[1][0], 2] + int(fill_ratio*(255/2))

        #print(mask)
        msg = Float32MultiArray()

        # This is almost always zero there is no empty padding at the start of your data
        msg.layout.data_offset = 0 

        # create two dimensions in the dim array
        msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        # dim[0] is the vertical dimension of your matrix
        msg.layout.dim[0].label = "channels"
        msg.layout.dim[0].size = 3
        msg.layout.dim[0].stride = 15
        # dim[1] is the horizontal dimension of your matrix
        msg.layout.dim[1].label = "samples"
        msg.layout.dim[1].size = 5
        msg.layout.dim[1].stride = 5

        msg.data = np.reshape(self.occupancy_grid,15)

        self.occupancy_grid_pub.publish(msg)

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


