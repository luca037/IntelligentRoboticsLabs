# Exercise: custom action

Similarly to homework 1, we have a roomba like robot. In this scenario we want to charge the robot using an action. 

| **Component**            | **Description**                                                                             |
| ------------------------ | ------------------------------------------------------------------------------------------- |
| **Scenario**             | Robot is already attached to the charging station with low battery (e.g., 5%)               |
| **Communication Method** | ROS Action                                                                                  |
| **Action Client**        | Charging Station - sends goal request with charging power ($p \leq 100$)                    |
| **Action Server**        | Robot - can accept or reject the charging request                                           |
| **Charging Process**     | If accepted, CS starts charging procedure                                                   |
| **Charging Duration**    | 1 minute to reach $min(100, b + p)$ where $b$ is current battery level                      |
| **Feedback**             | Robot continuously publishes current battery level to action feedback topic during charging |
| **Completion**           | Robot sends result to CS when battery reaches $min(100, b+p)$.                              |

In the provided solution we assume that the goal is always accepted by the server. The starting battery of the robot is a random number between 0 and 8. The power supplied by the charger is a random number between 80 and 100.

