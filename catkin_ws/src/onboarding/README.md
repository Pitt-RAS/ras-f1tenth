In this simple onboarding package, you will make a node that does three things:
* Subscribes to the "/scan" topic
* Figures out the min and max value from the data sent over the topic along with their respective angles
* Publishes to two topics "/min_point" and "/max_point" with their respective distance and corresponding angles

For this task you will need to know how the information is encoded in the <a href="http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html">sensor_msgs::LaserScan</a> object.<br>
The boiler plate source can be found in <a href="https://github.com/Pitt-RAS/ras-f1tenth/tree/main/catkin_ws/src/onboarding/node">node/MinMaxDistance.cc</a>.


Make sure you forked repository has https://github.com/Pitt-RAS/ras-f1tenth set as the upstream <br>
```$ git remote -v```

If you don't see
```
upstream https://github.com/Pitt-RAS/ras-f1tenth (fetch)
upstream https://github.com/Pitt-RAS/ras-f1tenth (push)
```
then run this command
```$ git remote add upstream https://github.com/Pitt-RAS/ras-f1tenth```

Update your the main branch in your fork<br>
```
$ git checkout main 
$ git pull upstream main
```

Checkout the onboarding branch
```
$ git checkout onboarding
```

Work on the task!

Be sure to commit as frequently as possible. When you're finished with the implementation
link your branch to one of the project leaders.
