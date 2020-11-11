# discretized movement ROS pacakge
Provides servers for actions that provide discretized wrappings for a
ROS MoveIt-compatible arm. The abilities of the discretized version of the arm
are defined by the action messages used to call the actions. For example, 
`move.msg` contains UP, DOWN, LEFT, and RIGHT, which can be passed to the
`direction` parameter of `Move.action`. 

The server can be started via `rosrun discretized_movement movement_server`.

