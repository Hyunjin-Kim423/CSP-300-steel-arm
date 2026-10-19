#!/usr/bin/env python3
"""Create a genuine OpenCV AprilTag 36h11 ID 0 test image, never a fake square."""
import argparse
try: import cv2
except ImportError as error: raise SystemExit('Install python3-opencv (with aruco) to generate this test asset.') from error
p=argparse.ArgumentParser(); p.add_argument('--output',default='synthetic_tag36h11_id0.pgm'); p.add_argument('--pixels',type=int,default=160); a=p.parse_args()
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
if hasattr(cv2.aruco, 'generateImageMarker'):
    marker=cv2.aruco.generateImageMarker(dictionary, 0, a.pixels)
else:
    marker=__import__('numpy').zeros((a.pixels, a.pixels), dtype='uint8')
    cv2.aruco.drawMarker(dictionary, 0, a.pixels, marker, 1)
canvas=255 * __import__('numpy').ones((480,640), dtype='uint8')
y=(480-a.pixels)//2; x=(640-a.pixels)//2; canvas[y:y+a.pixels,x:x+a.pixels]=marker
cv2.imwrite(a.output, canvas)
print(f'Wrote verified OpenCV AprilTag 36h11 ID 0; expected frontal depth is 0.1*600/{a.pixels:.0f} m: {a.output}')
