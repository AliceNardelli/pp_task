import numpy as np
import cv2 
import signal 
import sys
from pynput import keyboard
  
rectangles={
    "area1":{"x":0,"y":0,"w":0,"h":0,"name":""},
    "area2":{"x":0,"y":0,"w":0,"h":0,"name":""},
    "area3":{"x":0,"y":0,"w":0,"h":0,"name":""},
    "area4":{"x":0,"y":0,"w":0,"h":0,"name":""},
}
areas=["area1","area2","area3","area4"]
init=True
def define_contours(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    blurred = cv2.GaussianBlur(src=gray, ksize=(3, 5), sigmaX=0.5)
    edged = cv2.Canny(blurred, 30, 200)
    contours, hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    print("Number of Contours found = " + str(len(contours)))
    i=0
    #cv2.drawContours(image, contours, -1, (0, 255, 0), 3) 
    for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 800):
                x, y, w, h = cv2.boundingRect(contour)
                image = cv2.rectangle(image, (x, y),  (x + w, y + h), (255, 0, 0), 8)
                try:
                    image = cv2.putText(image, areas[i], (x,y),  cv2.FONT_HERSHEY_SIMPLEX,  1, (255, 0, 0) , 2, cv2.LINE_AA)
                    rectangles[areas[i]]["x"]=x
                    rectangles[areas[i]]["y"]=y
                    rectangles[areas[i]]["w"]=w
                    rectangles[areas[i]]["h"]=h
                    i+=1
                except:
                    print("more than needd are found")

                
    return image

    
def green_detection(imageFrame):
    # Convert the imageFrame in  
    # BGR(RGB color space) to  
    # HSV(hue-saturation-value) 
    # color space 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

    # Set range for green color and  
    # define mask 
    green_lower = np.array([25, 52, 72], np.uint8) 
    green_upper = np.array([102, 255, 255], np.uint8) 
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
  
  
    # Morphological Transform, Dilation 
    # for each color and bitwise_and operator 
    # between imageFrame and mask determines 
    # to detect only that particular color 
    kernel = np.ones((5, 5), "uint8") 
      

    # For green color 
    green_mask = cv2.dilate(green_mask, kernel) 
    res_green = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = green_mask) 
           
  
    # Creating contour to track green color 
    contours, hierarchy = cv2.findContours(green_mask, 
                                           cv2.RETR_TREE, 
                                           cv2.CHAIN_APPROX_SIMPLE) 
      

    return contours           
    #return imageFrame

def red_detection(imageFrame):
    # Convert the imageFrame in  
    # BGR(RGB color space) to  
    # HSV(hue-saturation-value) 
    # color space 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

    # Set range for green color and  
    # define mask 
    green_lower = np.array([136, 87, 111], np.uint8) 
    green_upper = np.array([180, 255, 255], np.uint8) 
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
  
  
    # Morphological Transform, Dilation 
    # for each color and bitwise_and operator 
    # between imageFrame and mask determines 
    # to detect only that particular color 
    kernel = np.ones((5, 5), "uint8") 
      

    # For green color 
    green_mask = cv2.dilate(green_mask, kernel) 
    res_green = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = green_mask) 
           
  
    # Creating contour to track green color 
    contours, hierarchy = cv2.findContours(green_mask, 
                                           cv2.RETR_TREE, 
                                           cv2.CHAIN_APPROX_SIMPLE) 
      

    return contours           
    #return imageFrame

def blue_detection(imageFrame):
    # Convert the imageFrame in  
    # BGR(RGB color space) to  
    # HSV(hue-saturation-value) 
    # color space 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

    # Set range for green color and  
    # define mask 
    green_lower = np.array([94, 80, 2], np.uint8) 
    green_upper = np.array([120, 255, 255], np.uint8) 
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
  
  
    # Morphological Transform, Dilation 
    # for each color and bitwise_and operator 
    # between imageFrame and mask determines 
    # to detect only that particular color 
    kernel = np.ones((5, 5), "uint8") 
      

    # For green color 
    green_mask = cv2.dilate(green_mask, kernel) 
    res_green = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = green_mask) 
           
  
    # Creating contour to track green color 
    contours, hierarchy = cv2.findContours(green_mask, 
                                           cv2.RETR_TREE, 
                                           cv2.CHAIN_APPROX_SIMPLE) 
      

    return contours           
    #return imageFrame




def key_press(key):
    global init
    print(f'Pressed {key}')
    init=False

    
def main(): 
    # Open the default webcam  
    cap = cv2.VideoCapture(2)
    listener = keyboard.Listener(on_press=key_press)
    listener.start()
    #detecting area while a keyboard is inserted
    while init: 
        # Read a frame from the webcam 
        ret, frame = cap.read() 
        if not ret: 
            print('Image not captured') 
            break
        bd = define_contours(frame)
        cv2.imshow("Edges", bd) 
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
        # Perform Canny edge detection on the frame
    #ordering areas
    x_values=[]
    reverse_dict_x={}
    reverse_dict_y={}
    y_values=[]
    for x in rectangles.keys():
        x_values.append(int(rectangles[x]["x"]))
        reverse_dict_x[int(rectangles[x]["x"])]=x
        y_values.append(int(rectangles[x]["y"]))
        reverse_dict_y[int(rectangles[x]["y"])]=x
    x_values.sort()  
    y_values.sort()
    x1=x_values[0]
    ar=reverse_dict_x[x1]
    y1=int(rectangles[ar]["y"])
    if y1==y_values[0] or y1==y_values[1]:
        print("find A1")
        rectangles[ar]["name"]="A1"
        ar2=reverse_dict_x[x_values[1]]
        rectangles[ar2]["name"]="A3"
    else:
        print("find A3")
        rectangles[ar]["name"]="A3"
        ar2=reverse_dict_x[x_values[1]]
        rectangles[ar2]["name"]="A1"
        
    
    x1=x_values[2]
    ar=reverse_dict_x[x1]
    y1=int(rectangles[ar]["y"])
    if y1==y_values[0] or y1==y_values[1]:
        print("find A2")
        rectangles[ar]["name"]="A2"
        ar2=reverse_dict_x[x_values[3]]
        rectangles[ar2]["name"]="A4"
    else:
        print("find A4")
        rectangles[ar]["name"]="A4"
        ar2=reverse_dict_x[x_values[3]]
        rectangles[ar2]["name"]="A2"
        
   
    

    while True:
         ret, frame = cap.read() 
         for x in rectangles.keys():
            print(x)
            ss=(int(rectangles[x]["x"]),int(rectangles[x]["y"]))
            se=(int(rectangles[x]["x"])+int(rectangles[x]["w"]), int(rectangles[x]["y"])+int(rectangles[x]["h"]))
            frame = cv2.rectangle(frame, ss,  se, (255, 0, 0), 8)
            frame = cv2.putText(frame, str(rectangles[x]["name"]), (int(rectangles[x]["x"]),int(rectangles[x]["y"])),  cv2.FONT_HERSHEY_SIMPLEX,  1, (255, 0, 0) , 2, cv2.LINE_AA)
         contours=red_detection(frame)
         for pic, contour in enumerate(contours):
                  area = cv2.contourArea(contour)
                  if(area > 800):
                         x, y, w, h = cv2.boundingRect(contour)
                         frame = cv2.rectangle(frame, (x, y),  
										(x + w, y + h), 
										(0, 0, 255), 8) 
                         for rect in rectangles.keys():
                              if (x>int(rectangles[rect]["x"])) and ((x+w)<(int(rectangles[rect]["x"])+int(rectangles[rect]["w"]))):
                                   if (y>int(rectangles[rect]["y"])) and ((y+h)<(int(rectangles[rect]["y"])+int(rectangles[rect]["h"]))):
                                        print("RED object inside "+str(rectangles[rect]["name"]))
				
         cv2.imshow("Edges", frame) 
        
         if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
        
    print("EXIT")  
    # Release the webcam and close the windows 
    cap.release() 
    cv2.destroyAllWindows()

    

if __name__ == "__main__": 
    main()




