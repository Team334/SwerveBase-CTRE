## Github
- Do this in command line (and push) after adding spotless checking to the github actions `git update-index --chmod=+x ./gradlew`.

## Camera Calibration
Extrinsics - Camera world-relative 3d pose. <br>
Intrinsics - Unique pin-hole camera properties (idk too much about this but it's things like focal length and optical center).

### How a point in world-coordinates is converted to a 2d image-coordinate
The point in world-coordinates is converted to camera-coordinates (camera is the origin) using the camera world-coordinates, also known as the extrinsics. Then, the point in camera-coordinates is converted to a 2d image-coordinate by applying the intrinsics on the point in camera-coordinates.

### Re-projection
Re-projection is done by the same exact process above. A point is converted from world-coordinates to pixel image-coordinates given the extrinsics and the intrinsics of the camera. Re-projection error is the distance in pixels between some re-projected world-coordinate point and that actual point in the frame in image-coordinates.

### SolvePNP
Given multiple points in world-coordinates, the camera's intrinsics, and the points in image-coordinates, solvePnP will find the camera's extrinsics. It will do this by guessing initial extrinsics, and then reprojecting all the world-coordinate points into image-coordinates. SolvePnP works iteratively to reduce the re-projection error between each image point and its corresponding re-projection. 

### Camera Calibration
To actually find intrinsics, camera calibration is necessary. A checkerboard pattern is used that is a set of corners with known world-coordinates (the top-left corner is the origin of the world-coordinate system). The calibrator will detect the corners in the image and map the image-coordinates of each corner to its world-coordinates. Many images at different positions of the pattern are taken, and an array of all the combined world-coordinate points and an array of all the combined image-coordinate points is made. Calibration works by guessing intrinsics initially, and then running solvePnP, to get a re-projection error that would be caused by incorrect intrinsics. The calibrator then works iteratively refining the intrinsics and calling solvePnP until the re-projection error is small enough, and at that point the intrinsics are found. The reprojection error for all the combined points can be used to determine how accurate the intrinsics really are. 

### Noise
todo

### Ambiguity
todo
