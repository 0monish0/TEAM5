import cv2
import numpy as np

img = cv2.imread('eye.png')

crop=img[500:700,0:300]

print(img.shape)
print(img.dtype)
print(type(img))
print(img)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ran = cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)

bla_img = np.zeros((500,500,1),dtype=np.uint8)
co_img = np.zeros((500,500,3),dtype=np.uint8)
cv2.rectangle(bla_img,(125,125),(375,375),(255),-1 )
cv2.circle(bla_img,(250,250),100,(0),2)
cv2.circle(co_img,(250,250),100,(1,255,225),4)
cv2.line(co_img,(250,100),(250,400),(255,125,125),2)
cv2.line(bla_img,(250,100),(250,400),(100),2)
cv2.putText(co_img,"KAMI",(125,125),cv2.FONT_HERSHEY_SIMPLEX,0.5,(125,125,255),2)

roi = img[0:300,0:300]
img[0:300,0:300]= [0,0,0]
flip_hori=cv2.flip(img,1)
flip_vert=cv2.flip(img,0)
cv2.imshow('hori',flip_hori)
cv2.imshow('vert',flip_vert)
resize = cv2.resize(img,None,fx=0.5,fy=0.5,interpolation=cv2.INTER_CUBIC)
cv2.imshow('resize',resize)
cv2.imshow('color',co_img)
cv2.imshow('black',bla_img)

print(bla_img)
print(bla_img.shape)
print(bla_img.dtype)


cv2.imshow('image',img)
cv2.imshow('crop',crop)
cv2.imshow('gray',gray)
cv2.imshow('ran',ran)
(h,w) = img.shape[:2]
center = (w/2,h/2)
M = cv2.getRotationMatrix2D(center,30,1.0)
rotateed = cv2.warpAffine(img,M,(w,h))
cv2.imshow('rotate',rotateed)


cv2.waitKey(0)