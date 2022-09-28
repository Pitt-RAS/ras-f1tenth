In this simple onboarding package, you will make a node that does three things:
* Subscribes to the "/scan" topic
* Figures out the min and max value from the data sent over the topic along with their respective angles
* Publishes to two topics "/min_point" and "max_point" with their respective distance and corresponding angles

Make sure you forked repository has https://github.com/Pitt-RAS/ras-f1tenth set as upstream <br>
```$ git remote -v```

If you don't see 
```
upstream https://github.com/Pitt-RAS/ras-f1tenth (fetch)
upstream https://github.com/Pitt-RAS/ras-f1tenth (push)
```
Then run this command
```
$ git remote add upstream https://github.com/Pitt-RAS/ras-f1tenth
```

