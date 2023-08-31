These simple scripts are from ROS tutorials. Just for test.

The `talker.py` publishes timestamp into `/chatter` that will be transferred by swarm_ros_bridge. And the `listener.py` listens to the `/chatter_recv` topic.

### Latency test method:

Connect two machines  to the same network segment.

In one machine:

1. modify the `neighbor` IP in `config/latency_test.yaml` to the other machine's IP.

2. ```bash
   roslaunch swarm_ros_bridge latency_test.launch

3. ```bash
   rosrun swarm_ros_bridge talker.py # in another terminal
   ```

4.  ```bash
    rosrun swarm_ros_bridge listener.py # in another terminal
    ```

In the other machine do the same thing as the first one.

Check the sending and receiving message time of two machines.

