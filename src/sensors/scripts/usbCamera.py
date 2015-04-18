#!/usr/bin/env python
import rospy
import cv_bridge
from cv_bridge import CvBridgeError
import cv
import time
import random
from sensor_msgs.msg import Image
import numpy as np

CAMERA_INDEX = 1

###################### GLOBAL SYSTEM STATE ########################

# class for a generic data holder
class Data:
    def __init__(self): pass    # empty constructor
    
# object (of class Data) to hold the global state of the system
D = Data()
D.STATE = "starting..."

# do we want to use a half-sized image?
D.half_size = False

# the game controller's buttons and pads ~ initial values...
D.BTNS = [0]*42
D.PADS = [0]*42


#################### INITIALIZATION FUNCTIONS ######################
def init_globals():
    """ sets up all the globals in the dictionary D
    """
    # get D so that we can change values in it
    global D

    # put threshold values into D
    D.thresholds    = {'low_red':0, 'high_red':255,\
                       'low_green':0, 'high_green':255,\
                       'low_blue':0, 'high_blue':255,\
                       'low_hue':0, 'high_hue':255,\
                       'low_sat':0, 'high_sat':255,\
                       'low_val':0, 'high_val':255 }

    # Set up the windows containing the image from the kinect,
    # the altered image, and the threshold sliders.
    cv.NamedWindow('threshold')
    cv.MoveWindow('threshold', 400, 0)
    cv.NamedWindow('image')
    cv.MoveWindow('image', 900, 0)
    cv.NamedWindow('sliders')
    cv.MoveWindow('sliders', 0, 0)

    # Create the sliders within the 'sliders' window
    cv.CreateTrackbar('low_red', 'sliders', D.thresholds['low_red'], 255, 
                          lambda x: change_slider('low_red', x) )
    cv.CreateTrackbar('high_red', 'sliders', D.thresholds['high_red'], 255, 
                          lambda x: change_slider('high_red', x) )
    cv.CreateTrackbar('low_green', 'sliders', D.thresholds['low_green'], 255, 
                          lambda x: change_slider('low_green', x) )
    cv.CreateTrackbar('high_green', 'sliders', D.thresholds['high_green'], 255, 
                          lambda x: change_slider('high_green', x) )
    cv.CreateTrackbar('low_blue', 'sliders', D.thresholds['low_blue'], 255, 
                          lambda x: change_slider('low_blue', x) )
    cv.CreateTrackbar('high_blue', 'sliders', D.thresholds['high_blue'], 255, 
                          lambda x: change_slider('high_blue', x) )
    cv.CreateTrackbar('low_hue', 'sliders', D.thresholds['low_hue'], 255, 
                          lambda x: change_slider('low_hue', x) )
    cv.CreateTrackbar('high_hue', 'sliders', D.thresholds['high_hue'], 255, 
                          lambda x: change_slider('high_hue', x) )
    cv.CreateTrackbar('low_sat', 'sliders', D.thresholds['low_sat'], 255, 
                          lambda x: change_slider('low_sat', x) )
    cv.CreateTrackbar('high_sat', 'sliders', D.thresholds['high_sat'], 255, 
                          lambda x: change_slider('high_sat', x) )
    cv.CreateTrackbar('low_val', 'sliders', D.thresholds['low_val'], 255, 
                          lambda x: change_slider('low_val', x) )
    cv.CreateTrackbar('high_val', 'sliders', D.thresholds['high_val'], 255, 
                          lambda x: change_slider('high_val', x) )

    # Set the method to handle mouse button presses
    cv.SetMouseCallback('image', onMouse, None)

    # We have not created our "scratchwork" images yet
    D.created_images = False

    # Variable for key presses - set it to something that could not have been pressed
    D.last_key_pressed = 255

    # Create a connection to the Kinect
    D.bridge = cv_bridge.CvBridge()
    D.camera = cv.CaptureFromCAM(CAMERA_INDEX)
    D.running = True

    # image-based values, including the font we need to draw text
    D.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, .4, .4, 0, 1)
    D.image = None # no image yet
    D.orig_image = None

    D.pixel_delta = 10



################## END INITIALIZATION FUNCTIONS ####################

################### IMAGE PROCESSING FUNCTIONS #####################
def threshold_image():
    """ runs the image processing in order to create a 
        black and white thresholded image out of D.image
        into D.threshed_image
    """
    # get D so that we can change values in it
    global D

    # Use OpenCV to split the image up into channels,
    # saving them in their respective bw images
    cv.Split(D.image, D.blue, D.green, D.red, None)

    # This line creates a hue-saturation-value image
    cv.CvtColor(D.image, D.hsv, cv.CV_RGB2HSV)
    cv.Split(D.hsv, D.hue, D.sat, D.val, None)

    # Here is how OpenCV thresholds the images based on the slider values:
    cv.InRangeS(D.red, D.thresholds["low_red"], \
                    D.thresholds["high_red"], D.red_threshed)
    cv.InRangeS(D.green, D.thresholds["low_green"], \
                    D.thresholds["high_green"], D.green_threshed)
    cv.InRangeS(D.blue, D.thresholds["low_blue"], \
                    D.thresholds["high_blue"], D.blue_threshed)
    cv.InRangeS(D.hue, D.thresholds["low_hue"], \
                    D.thresholds["high_hue"], D.hue_threshed)
    cv.InRangeS(D.sat, D.thresholds["low_sat"], \
                    D.thresholds["high_sat"], D.sat_threshed)
    cv.InRangeS(D.val, D.thresholds["low_val"], \
                    D.thresholds["high_val"], D.val_threshed)

    # Multiply all the thresholded images into one "output" image,
    # named D.threshed_image
    cv.Mul(D.red_threshed, D.green_threshed, D.threshed_image)
    cv.Mul(D.blue_threshed, D.threshed_image, D.threshed_image)
    cv.Mul(D.hue_threshed, D.threshed_image, D.threshed_image)
    cv.Mul(D.sat_threshed, D.threshed_image, D.threshed_image)
    cv.Mul(D.val_threshed, D.threshed_image, D.threshed_image)

    # Erode and Dilate shave off and add edge pixels respectively
    #cv.Erode(D.threshed_image, D.threshed_image, iterations = 1)
    #cv.Dilate(D.threshed_image, D.threshed_image, iterations = 1)


def find_biggest_region():
    """ finds all the contours in threshed image, finds the largest of those,
        and then marks in in the main image
    """
    # get D so that we can change values in it
    global D

    cv.Copy( D.threshed_image, D.copy ) # copy threshed image

    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(D.copy, D.storage, cv.CV_RETR_EXTERNAL, \
                                   cv.CV_CHAIN_APPROX_SIMPLE)

    # Next we want to find the *largest* contour
    if len(contours) == 0:
        print "No regions found"
        return

    # there _were_ contours found - let's get the largest...
    biggest = contours
    biggestArea = cv.ContourArea(contours)
    while contours != None:
        nextArea = cv.ContourArea(contours)
        if biggestArea < nextArea:
            biggest = contours
            biggestArea = nextArea
        contours = contours.h_next()
    
    # Use OpenCV to get a bounding rectangle for the largest contour
    br = cv.BoundingRect(biggest, update=0)

    #print "in find_regions, br is", br

    # Example of drawing a red box
    # Variables: ulx ~ upper left x, lry ~ lower right y, etc.
    ulx = 142
    lrx = 200
    uly = 142
    lry = 150
    cv.PolyLine(D.image, [[(ulx,uly), (lrx,uly), 
                           (lrx,lry), (ulx,lry)]], 
                            2, # thickness 
                            cv.RGB(255, 0, 0)) # color == red


    # Example of drawing a yellow circle
    # Variables: cenx, ceny
    cenx = 100
    ceny = 100
    cv.Circle(D.image, (cenx,ceny), 8, 
                        cv.RGB(255, 255, 0), # color == yellow
                        thickness=1, lineType=8, shift=0)



def draw_text_to_image():
    """ A function to handle drawing things to the image """
    # draw a rectangle under the text to make it more visible
    cv.Rectangle(D.image, (0,0), (100,50), cv.RGB(255,255,255), cv.CV_FILLED)
    # place some text there
    D.STATE = "hi!" # probably not the state name you want...
    cv.PutText(D.image, D.STATE, (5,10), D.font, cv.RGB(0,0,0))
    # and some values
    randomness = random.randint(-100,100)
    area = 4242 + randomness
    area_string = str(area)
    cv.PutText(D.image, area_string, (5,25), D.font, cv.RGB(0,0,0))





################# END IMAGE PROCESSING FUNCTIONS ###################

####################### CALLBACK FUNCTIONS #########################
def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """

    if event==cv.CV_EVENT_LBUTTONDOWN: # clicked the left button
        print "x, y are", x, y
        (b,g,r) = D.image[y,x]
        print "r,g,b is", int(r), int(g), int(b)
        (h,s,v) = D.hsv[y,x]
        print "h,s,v is", int(h), int(s), int(v)

    if event==cv.CV_EVENT_RBUTTONDOWN: # clicked the right button
        print "\n\n  *** Right button click! *** \n\n"
        # you'll want to use this to start a model of the pixel
        # colors you'd like to track...
        # first, print the r,g,b and h,s,v at the right-clicked point...


def check_key_press(key_press):
    """ this handler is called when a real key press has been
        detected, and updates everything appropriately
    """
    # get D so that we can change values in it
    global D
    D.last_key_pressed = key_press


    if key_press == ord('q') or key_press == 27: # if a 'q' or ESC was pressed
        time.sleep(0.1)  # pause for a moment...    # then, shutdown:
        rospy.signal_shutdown( "Quitting..." )
        return

    # firing takes a few seconds, so you'll need to wait ...
    elif key_press == ord('\n'):
            D.launchcontrol.fire()
    elif key_press in [ord('z'), ord(' ')]:
            D.launchcontrol.stop()
        
    elif key_press == ord('S'):  # save to file
        x = D.thresholds   # the value we will save
        f = open( "./thresh.txt", "w" )   # open the file "thresh.txt" for writing
        print >> f, x   # print x to the file object f
        f.close()   # it's good to close the file afterwards
        print "Saving current slider threshold values..."
        
    elif key_press == ord('L'):  # load from file
        try:
            f = open( "./thresh.txt", "r" )   # open the file "thresh.txt" for reading
            data = f.read()   # read everything from f into data
            x = eval( data )  # eval is Python's evaluation function
            f.close()   # it's good to close the file afterwards
            D.thresholds = x
            cv.SetTrackbarPos('low_red', 'sliders', D.thresholds['low_red'])
            cv.SetTrackbarPos('high_red', 'sliders', D.thresholds['high_red'])
            cv.SetTrackbarPos('low_green', 'sliders', D.thresholds['low_green'])
            cv.SetTrackbarPos('high_green', 'sliders', D.thresholds['high_green'])
            cv.SetTrackbarPos('low_blue', 'sliders', D.thresholds['low_blue'])
            cv.SetTrackbarPos('high_blue', 'sliders', D.thresholds['high_blue'])
            cv.SetTrackbarPos('low_hue', 'sliders', D.thresholds['low_hue'])
            cv.SetTrackbarPos('high_hue', 'sliders', D.thresholds['high_hue'])
            cv.SetTrackbarPos('low_sat', 'sliders', D.thresholds['low_sat'])
            cv.SetTrackbarPos('high_sat', 'sliders', D.thresholds['high_sat'])
            cv.SetTrackbarPos('low_val', 'sliders', D.thresholds['low_val'])
            cv.SetTrackbarPos('high_val', 'sliders', D.thresholds['high_val'])
            print "Loaded saved slider threshold values..."
        except:
            print "No file named thresh.txt found!"
        
    else:
        print "Did not recognize a command for key #", key_press, "(", chr(key_press), ")"


def change_slider(name, new_threshold):
    """ changes the slider values given the name of the slider and the new value """
    # get D so that we can change values in it
    global D
    D.thresholds[name] = new_threshold


def init_images():
    """ Creates all the images we'll need. Is separate from init_globals 
        since we need to know what size the images are before we can make
        them
    """
    # get D so that we can change values in it
    global D

    # Find the size of the image 
    # (we set D.image right before calling this function)
    D.img_size = cv.GetSize(D.image)

    # Create images for each color channel
    D.red = cv.CreateImage(D.img_size, 8, 1)
    D.green = cv.CreateImage(D.img_size, 8, 1)
    D.blue = cv.CreateImage(D.img_size, 8, 1)
    D.hue = cv.CreateImage(D.img_size, 8, 1)
    D.sat = cv.CreateImage(D.img_size, 8, 1)
    D.val = cv.CreateImage(D.img_size, 8, 1)

    # Create images to save the thresholded images to
    D.red_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.green_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.blue_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.hue_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.sat_threshed = cv.CreateImage(D.img_size, 8, 1)
    D.val_threshed = cv.CreateImage(D.img_size, 8, 1)

    # Create storage for image processing later on...
    D.storage = cv.CreateMemStorage(0) # Create memory storage for contours
    D.copy = cv.CreateImage(D.img_size, 8, 1)

    # The final thresholded result
    D.threshed_image = cv.CreateImage(D.img_size, 8, 1)

    # Create the hsv image
    D.hsv = cv.CreateImage(D.img_size, 8, 3)



##################### END CALLBACK FUNCTIONS #######################

def main():
    """ the main program that sets everything up
    """
    # initialize publisher to the imageStream topic. Must publish string
    # messages.
    pub = rospy.Publisher('imageStream', Image, queue_size = 10)
        
    # Initialize our node
    rospy.init_node('blobFinder')

    # Initialize all the global variables we will need
    init_globals()

    # our main loop!
    while rospy.is_shutdown() == False:

        # Grab incoming image
        D.orig_image = cv.QueryFrame(D.camera)
        # handle half_size
        if D.half_size == False: # keep the image size
            D.image = D.orig_image
        else: # halve the image size
            if D.created_images == False:
                w, h = D.orig_image.width, D.orig_image.height
                D.half_sz = (w/2,h/2)
                D.image = cv.CreateImage(D.half_sz, 8, 3)
            cv.Resize(D.orig_image,D.image)

        # Get the incoming image from the Kinect
        #D.image = D["bridge"].imgmsg_to_cv(data, "bgr8")

        if D.created_images == False:
            # Initialize the additional images we need for processing
            # We only need to run this one time
            # But, sometimes it's not ready before the while loop starts...
            init_images()
            D.created_images = True

        # Recalculate threshold image
        threshold_image()

        # Recalculate blob in main image
        find_biggest_region()

        # draw any text on the image
        draw_text_to_image()

        # Get any incoming keypresses
        # To get input from keyboard, we use cv.WaitKey
        # Only the lowest eight bits matter (so we get rid of the rest):
        key_press_raw = cv.WaitKey(5) # gets a raw key press
        key_press = key_press_raw & 255 # sets all but the low 8 bits to 0
        
        # Handle key presses only if it's a real key (255 = "no key pressed")
        if key_press != 255:  check_key_press(key_press)

        # Update the displays:
        # Main image:
        cv.ShowImage('image', D.image)
        # print type(D.image)

        # Currently selected threshold image:
        cv.ShowImage('threshold', D.threshed_image )
        
        # for some reason, the cv->ROS bridge requires a np array and not
        # an iplimage. asarray is a faster (and more reliable) way to convert 
        # to an n-dimensional array.        
        imageArr = np.asarray(D.image[:,:])
                
        # convert the OpenCV image to a ROS message.
        try:
            image_message = D.bridge.cv2_to_imgmsg(imageArr, "bgr8")
            # publish the image_message to the imageStream topic.
            pub.publish(image_message)

        except CvBridgeError, e:
            print e
            
        
        
    # here, the main loop has ended
    print "Shutting down..."


# this is the "main" trick: it tells Python
# what code to run when you run this as a stand-alone script:
if __name__ == "__main__":
    main()
