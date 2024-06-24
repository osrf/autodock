This package consolidates all configs related to the `kabam` project. The configs are just kept private here. The actual algorithm is open sourced here: https://github.com/osrf/autodock

### Run on Vacuum40

Run the `autodock_node` on the `Vacuum40` robot. 
Please make sure that the remote rosmaster time is in-synced.

```bash
export ROS_HOSTNAME=10.7.5.155 # your local ip
export ROS_MASTER_URI=http://10.7.5.88:11311
roslaunch autodock_kabam vacuum40.launch
```

**Faking ros clock with sim time (Caution)**

Here, the `autodockServer` node is running on a secondary machine, sharing the 
same roscore of the host. However here the host's time is not in sync with the 
secondary PC. (due to NTP server issue, and we cant ssh into the primary PC). 
Therefore to solve this headache, we will be publish the republish the primary pc 
clock to `clock`, this is handled by the `camera_info_filter.py` node. Then, 
enable `auto_docking_server` "fake clock" mode by adding a `--fake_clock` arg tag. 
Under the hood, this enables `use_sim_time` to only `auto_docking_server`.

---

### Run on Oscar

Add `gaussian 10.7.5.88` to `/etc/hosts`

```bash
export ROS_HOSTNAME=10.7.5.165 # your local ip
export ROS_MASTER_URI=http://10.7.5.88:11311
roslaunch autodock_kabam oscar.launch
```

> Hack: User might face ros-time off-synced issue, this can be avoided by enabling `use_fake_time`:

```bash
roslaunch autodock_kabam oscar.launch use_fake_clock:=1
```

Here, an external `obstacle_observer` node is used to pause when obstacle is detected. 
The robot's `/local_costmap` is being used here.

Similarly, a dock test script is used here as a smoke test. It will `cmd_vel` the robot randomly 
in front of the charger and dock it repeatedly.
```bash
# indicate the number of attempt with the -c arg
# indicate charging station srv name with -cs
rosrun autodock_examples dock_robot_test.py -c 10 -cs /xnergy_charger_rcu/trigger_charging
```