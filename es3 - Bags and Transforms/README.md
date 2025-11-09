# Bags and Transformations

| **Component**        | **Description**                                                                                                                                                                    |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Scenario**         | Vacuum Cleaner (VC) ran out of power overnight and stopped far from Charging Station (CS)                                                                                          |
| **Detection Method** | Both VC and CS are marked with **AprilTags** detected by a service camera on the ceiling                                                                                           |
| **Data Source**      | Recorded movement data available in **bag_exercise3.zip**                                                                                                                          |
| **Main Task**        | Develop a node that processes the bag file to locate VC relative to CS                                                                                                             |
| **Key Objectives**   | - Find VC path and CS position relative to `tag36h11:0`<br>- Project data onto z-axis plane of `tag36h11:0`<br>- Find VC's path with respect to CS<br>- Save path data in CSV file |
| **Visualization**    | Create Python script to plot the path, highlighting final VC position and CS location                                                                                              |
| **AprilTag IDs**     | - `tag36h11:0`: Floor plane reference<br>- `tag36h11:1`: Charging Station (CS)<br>- `tag36h11:2`: Vacuum Cleaner (VC)                                                              |
| **Tools**            | Use `rviz2` for visualization (configure to show messages with past timestamps)                                                                                                    |


## Execute the code

Open two terminal, in one execute:

```shell
ros2 bag play es_bag2.mcap
```

In the other one:

 ```shell
ros2 run bag_listener listener
 ```

Note that the listener will not stop executing. You will need to manually stop it with `CTRL-C` when you notice that the output doesn't change any more.
