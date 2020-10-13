# Architecture 3T

```console
rosrun turtlesim turtlesim_node
rosrun architecture_3t planning _circle_time_sec:=10.0
rosrun architecture_3t executive
rosservice call /planning/order/create "order:
  num_circles: 2"
```
