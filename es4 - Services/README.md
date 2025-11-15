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

The code is open-source and can be found at LINK.

TODO: write the part related to the detection of the apples, the main steps performed with computer vision!
