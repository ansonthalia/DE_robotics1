import numpy as np
import argparse
import cv2

class Camera:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
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
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

    def take_photo():

        # Initialize
        rospy.init_node('take_photo', anonymous=False)
        camera = TakePhoto()

        # Take a photo
        img_title = 'head_view'

        if camera.take_picture(img_title):
            rospy.loginfo("Saved image " + img_title)
        else:
            rospy.loginfo("No images received")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)
        return img_title

#directory = 'supreme.png' --This was used to test 
DENIRO_cam = Camera()
get_image = DENIRO_cam.take_photo()

#Diretory found manually



def colour_detect(directory):
    
    #define image directory
    directory = 'Camera/head_view.png'
    # load the image
    image = cv2.imread(directory)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # create NumPy arrays for boundaries. These need to be calibrated. 
    lower = np.array([0,30,0])
    upper = np.array([255,255, 255])

    # find the colors within the specified boundaries and apply the mask
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)

    # show the images for debugging
    cv2.imshow("images", np.hstack([image, output]))
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
