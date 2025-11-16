# Services

| **Component**            | **Description**                                                                                                                                                                            |
| :----------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Scenario**             | A TurtleBot robot and its burrow communicate about resource management.                                                                                                                    |
| **Communication Method** | ROS Service                                                                                                                                                                                |
| **Service Client**       | Burrow Node - Sends requests containing:<br>- `n`: current number of apples (resources)<br>- `s`: total size/capacity of the burrow<br>- Constraint: `n < s` (randomly generated)          |
| **Service Server**       | TurtleBot Node - Receives requests and sends a boolean response:<br>- `True`: if enough apples are found to refill the burrow to capacity `s`<br>- `False`: if not enough apples are found |
| **TurtleBot Sensor**     | LIDAR - Used to detect apples (represented as spheres in the simulation).                                                                                                                  |
| **TurtleBot Publisher**  | Publishes the poses of detected apples to the `/apples` topic (`geometry_msgs/msg/PoseArray`), relative to its `base_link`.                                                                |
| **Testing Requirement**  | Code must explore multiple scenarios:<br>- `n < s`<br>- `n = s`<br>- Enough apples found vs. not enough apples found                                                                       |
| **Visualization**        | All communication phases must be shown to the user via terminal printouts.                                                                                                                 |

## Execute the code

First compile and source. Then run:

```shell
ros2 launch group_24_ex4 ex4.launch.py
```

## Solution

The code is open-source and can be found at [link](https://github.com/luca037/IntelligentRoboticsLabs/tree/main/es4%20-%20Services)

The strategy used to detect the apples can be summarized in the following steps:

1. **Reconstruct the lidar map in an image**
	- Transform the polar coordinates in `LaserScan.msg` into Cartesian coordinates (expressed w.r.t. the lidar reference frame).
	- Transform the Cartesian coordinates into pixels coordinates.
2. **Remove walls from the image** using Hough line transform.
3. **Detect circles** using Hough circle transform and save the *centers*.
4. Transform the coordinates of the *centers* from pixel to Cartesian.
