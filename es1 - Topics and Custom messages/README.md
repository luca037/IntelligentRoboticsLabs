# Exercise: custom message

We have a roomba like robot that needs to clean the rooms of a building.

| **Component**                      | **Description**                                                                                                                                                  |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Custom Message (Msg)               | Create a ROS2 message type containing: <br> - Room ID<br> - Room name<br> - Battery level                                                                        |
| Robot Node (Publisher)             | - Publishes robot data to a topic at **5Hz** <br> - Data includes: room ID, room name, battery level <br> - Topic name: `/charge_status`                         |
| Charging Station Node (Subscriber) | - Subscribes to the `/charge_status` topic at **5Hz** <br> - Prints received messages (room ID, name, battery) to the terminal                                   |
| Room Details                       | Use the provided lab names and IDs: <br> - 1: Robot Vision Lab <br> - 2: SSL Lab <br> - 3: Neurorobotics Lab <br> - 4: IAS-Lab <br> - 5: Autonomous Robotics Lab |
| Topic Frequency                    | Both publisher and subscriber operate at **5Hz**                                                                                                                 |
| Visual/Graph Structure             | System follows the structure: <br> `/robot` → (publishes) → `/charge_status` → (received by) → `/charging_station`                                               |

The logic regarding battery drain, the choice of the room to clean and the cleaning time are up to you.

What i've done:

- Battery drains 1% every second (start from 100%).
- Room to clean selected at random.
- Time to clean a room: 3 seconds.
