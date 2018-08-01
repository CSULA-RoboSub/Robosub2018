'''
Script to convert images labeled with Sloth into a json file into a folder of cropped images
around the bounding boxes

Usage:
At command line:
python [path to images folder] [path where cropped images will be stored] [path to json file]
'''


import json
import os
import sys

import cv2
import numpy as np 

#convenience method to convert an array of annotation dicts found  in the json file
#to tuples of coordinates for a bounding rectangle, (x1, y1, x2, y2)
def annotationsToTuples(annotationArray):
	result = [(a['x'], a['y'], a["width"], a["height"]) for a in annotationArray]
	return result


#returns a dictionary of imagepaths -> array of bounding boxes 
def getBoundingBoxesForImages(jsonData):
	boxes = {}
	result = {x["filename"]: annotationsToTuples(x["annotations"]) for x in jsonData}
	return result

#gets a list of sign images from scenes containing signs and json file of labels
def getImagesFromJSON(jsonString, imgdir=os.getcwd()):
	signs = []
	boxes = getBoundingBoxesForImages(jsonString)
	for key, arr in boxes.items():
		scene = cv2.imread(os.path.join(imgdir, key))
		if scene is None:
			print "{} is either not found or not readable".format(key)
			continue
		for coord in arr:
			x, y, width, height = max(0, int(coord[0])), max(0, int(coord[1])), max(0, int(coord[2])), max(0, int(coord[3]))
			if width == 0 or height == 0: 
				continue			
			img = scene[y:y+height, x:x+width, :]
			signs.append((key, img))
	return signs

def main():
	image_dir = sys.argv[1]
	to_dir = sys.argv[2]
	json_file = sys.argv[3]

	if not os.path.isdir(image_dir):
		print "{} not found".format(image_dir)
		return

	if not os.path.isdir(to_dir):
		print "{} not found".format(to_dir)
		return

	if not os.path.exists(json_file):
		print "{} not found".format(json_file)
		return

	with open(json_file) as json_file: 
		jsonData = json.load(json_file)

	images = getImagesFromJSON(jsonData, image_dir)
	for file, image in images:
		print "Attempting to write {}".format(os.path.join(to_dir, "cropped_" + file))
		cv2.imwrite(os.path.join(to_dir, "cropped_" + file), image)


if __name__ == "__main__":
	main()









	
	