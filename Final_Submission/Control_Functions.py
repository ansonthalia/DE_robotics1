#-------------------Control Functions---------------------
'''This file is a selection of functions called by House_of_Cards() to add some robustness to the process of
building the structure.These functions allow the system to know if issues have arisen during the brick moving
process. The system can know if it drops a Brick or if the Structural Fails.'''

import numpy as np
import argparse
import cv2

class DENIRO_camera:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/cameras/head_camera/image"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image in folder 'Control_Functions'
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

def take_photo():

    # Initialize
    rospy.init_node('take_photo', anonymous=False)
    DENIRO_camera = TakePhoto()

    # Take a photo
    img_title = 'head_view'

    if DENIRO_camera.take_picture(img_title):
        rospy.loginfo("Saved image " + img_title)
    else:
        rospy.loginfo("No images received")

    # Sleep to give the last log messages time to be sent
    rospy.sleep(1)
    return img_title

#directory = 'supreme.png' --This was used to test colout_detect

def colour_detect():
    '''
    colour_detect() function is used to calculate the percentage
    brick colour in the head_camera image. It uses the images from 
    the take_photo() function above which takes photos using the head_camera
    This is used to check to see if the structure has fallen.
    '''
    #define image directory
    directory = '~/Control_Functions/head_view.png'
    # load the image
    image = cv2.imread(directory)
    #convert to HSV for better colour detect
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # create NumPy arrays for boundaries. These need to be calibrated. 
    lower = np.array([0,30,0])
    upper = np.array([255,255, 255])

    # find the colors within the specified boundaries and apply the mask
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)

    # show the images for debugging
    #cv2.imshow("images", np.hstack([image, output]))
    
    #Calculations for percentage brick colour
    height, width = mask.shape[:2]  
    pixel_no = float(height*width)
    count = float(0)

    # Calculate % brick colour
    for j in range(0, width-1):
        for i in range(0, height-1):
            if mask[i][j] != 0:
                count += 1

    print('Count = %i'%count)    
    print('pixel_no = %i'%pixel_no)     
    percentage = count*100/pixel_no
    cv2.waitKey(0)

    return percentage
