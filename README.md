# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Data Buffer
### MP.1 Data Buffer Optimization
Create a ring buffer in FIFO by using vector data structure. using a while loope to limit the size of the buffer.


### MP.2 Keypoint Detection
Create two funcitons for keypoints detection. one for Harris (detKeypointsHarris) and the other (detKeypointsModern) for  FAST, BRISK, ORB, AKAZE, and SIFT.
by using detectortype as funciton parameter to make it selectable and using string comparison to create the relevant keypoint detector accordingly.

### MP.3 Keypoint Removal
Use a for loop to traverse all potential keypoints and check if it locate inside the predefine rect ROI.
create a temporary vector with those keypoints inside roi and replace that keypoints object at the end.
sum up detected keypoints in ROI for all 10 pics

### MP.4 Keypoint Descriptors
Create function descKeypoints to perform keypoint description.
by using descriptorType as funciton parameter to make it selectable and using string comparison to create the relevant keypoint extractor accordingly.
It is not possible to combine AKAZE descriptors with other detector due to terminating with uncaught exception


### MP.5 Descriptor Matching
Implement function matchDescriptors with descriptorType, matcherType and selectorType as parameters to combine them accordingly. As SIFT output is float type, L2 normination has to be used in case of brute force marching. otherwise use hamming normination for binary output. 
sum up matched keypoints in ROI for all 10 pics.

### MP.6 Descriptor Distance Ratio
Ignore those associated pair of keypoints whose best matching and second best matching are too close to each other.

## Performance 

### Evaluation 1: Count the number of keypoints on the preceding vehicle for all 10 images 

| Detector   | Total Keypoints |
| ---------|:-------------:| 
| HARRIS | 248  | 
| FAST   | 4094 | 
| BRISK  | 2762 |
| ORB    | 1161 |
| AKAZE  | 1670 | 
| SIFT   | 1386 |



 ### Evaluation 2: Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8. 


| Detector + Descriptor  | Total Keypoints | Matched Keypoints  | Total Time (ms)  |
| ------------------------|:---------------:| ------------------:|-----------------:|
| SHITOMASI + BRISK |1179|767|301.8|
| SHITOMASI + BRIEF|1179|944|137.3 |
| SHITOMASI + ORB|1179 |907|144.8|
| SHITOMASI + FREAK|1179 |766|324.7|
| SHITOMASI + AKAZE|error|error|error|
| SHITOMASI + SIFT|1179|927|263.2|
| HARRIS + BRISK|248|142|344.7|
| HARRIS + BRIEF|248|173 |191.1 |
| HARRIS + ORB|248|160 |183.8 |
| HARRIS + FREAK|248|146 |356.2 |
| HARRIS + AKAZE|error|error|error|
| HARRIS + SIFT|248|163|290.9|
| FAST + BRISK|4094|2183 |279.1 |
| FAST + BRIEF |4094|2831 |88.3 |
| FAST + ORB |4094|2762|121.7 |
| FAST + FREAK|4094|2225 |284.3 |
| FAST + AKAZE|error|error|error|
| FAST + SIFT|4094|2782|356.6|
| BRISK + BRISK |2762|1570|858.8|
| BRISK + BRIEF |2762|1704|708.8 |
| BRISK + ORB |2762|1510|640.7 |
| BRISK + FREAK |2762|1526|770.7 |
| BRISK + AKAZE|error|error|error|
| BRISK + SIFT|2762|1646|1037.9|
| ORB + BRISK |1161|751 |374.8|
| ORB + BRIEF|1161|545 |146.5|
| ORB + ORB |1161|761 |209.6|
| ORB + FREAK|1161|421 |325.7 |
| ORB + AKAZE|error|error|error|
| ORB + SIFT|1161|763|406.2|
| AKAZE + BRISK|1670|1215 |1015.3 |
| AKAZE + BRIEF|1670|1266 |810.5 |
| AKAZE + ORB|1670|1186 |1181.1 |
| AKAZE + FREAK |1670|1188 |895.8 |
| AKAZE + AKAZE|1670|1259 |1030.7 |
| AKAZE + SIFT|1670|1270|854.3|
| SIFT + BRISK|1386|592 |1392.4 |
| SIFT + BRIEF|1386|702 |1035.4 |
| SIFT + FREAK|1386|596 |1270.8 |
| SIFT + AKAZE|error|error|error|
| SIFT + SIFT|1386|800|1983.9|

### Evaluation 3: based on the benchmark above, the TOP3 detector & descriptor combinations are:

| Detector + Descriptor  | Total Keypoints | Matched Keypoints  | Total Time (ms)  |
| ------------------------|:---------------:| ------------------:|-----------------:|
| FAST + BRIEF |4094|2831 |88.3 |
| FAST + ORB |4094|2762|121.7 |
| SHITOMASI + BRIEF|1179|944|137.3 |