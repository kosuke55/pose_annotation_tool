# point_cloud_annotation_tool
Original code is https://github.com/himlen1990/toolbox/tree/master/annotation_tool  

This interface is built upon librviz, for more detail please check http://docs.ros.org/indigo/api/librviz_tutorial/html/index.html

Usage
====

0.build the package under ros workspace.

1.prepare your point cloud dataset.

2.click load point cloud directory button and open your dataset direction.
(default point cloud format is ply and PointXYZRGB, you can change it to pcd by editing the source code).
![image](https://github.com/himlen1990/toolbox/blob/master/annotation_tool/IMG/1.png)
![image](https://github.com/himlen1990/toolbox/blob/master/annotation_tool/IMG/2.png)

3.click add marker and start annotation (you can also change the marker type by clicking swith marker). 
![image](https://github.com/himlen1990/toolbox/blob/master/annotation_tool/IMG/3.png)

4.click save label and move to next frame after you finishing annotate the current frame.

5.you can jump to arbitrary frame by giving a frame number and click move to frame.

6.before click "load annotation for checking", you should asign the point cloud directory first (follow step2)

## Utils
### [make_circle_points.py](utils/make_circle_points.py)
Annotate the base pose  
<img src="https://user-images.githubusercontent.com/39142679/103482335-a8569300-4e23-11eb-9df5-dfd37f92446e.png" width="800">  
Generate a circular annotation of the zy plane centered on the base pose by specifying the radius and spacing.  
Check annotation before overwriting with -g option.

```
ipython -i --  make_circle_points.py -i /home/kosuke55/catkin_ws/src/pose_annotation_tool/real_ycb_annotation_pouring/029_plate_0.txt --radius 0.08 --interval 0.01 -g
```

<img src="https://user-images.githubusercontent.com/39142679/103482334-a7bdfc80-4e23-11eb-81b1-7bb22865e721.png" width="300">  
If there is no problem, overwrite the original annotation with the -r option.

```
ipython -i --  make_circle_points.py -i /home/kosuke55/catkin_ws/src/pose_annotation_tool/real_ycb_annotation_pouring/029_plate_0.txt --radius 0.08 --interval 0.01 -r
```
Load generated annotation poses.  
<img src="https://user-images.githubusercontent.com/39142679/103482333-a68ccf80-4e23-11eb-9968-c1ae67987f82.png" width="800">
