# GraspingHelper
Utilizing realsense libraries and PCL to segment and localize cylinders on the desktop. Using Realsense D-series RGB-D camera. 

## Data
* Capture 45 frames via a moving depth camera
* Convert them into .pcd files
* Below is an example showing the raw PointCloud data
* video:
<a href="https://youtu.be/2uXk4LdyCR4"><img src="results_demo/raw.png" width="60%" /></a>

## Implementation
* Utilizing number of cylinders provided by user in advance to save computation time and enhance accuracy;

* StatisticalFilter can save about 0.02-0.04s and enhance accuracy as well;

* 'LeafSize' in voxelization is quite powerful for save computation time;

* RegionGrowing does not work well in my case. Probably because of collections of outliers are hard to remove
 such that they are all counted into results;
=======
* RegionGrowing does not work well in my case. Probably because collections of outliers are hard to remove which are all counted into results;


## Results
* video: 
<a href="https://youtu.be/-os6dyEvwH4"><img src="results_demo/cylinders.png" width="60%" /></a>
