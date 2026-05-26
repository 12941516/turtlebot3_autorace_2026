# turtlebot3_autorace_2026
Repository for Turtlebot3 AUTORACE Contest 2026, In AJOU Univ. ATOM

## To Run
1. run camera publisher

```bash
~$ ros2 run cv2_examples video_publisher
```

2. run detection node for each mission

```bash
~$ ros2 run [MISSION] detection
```

3. run trigger command
```bash
~$ ros2 topic pub -1 /[MISSION]_state std_msgs/Bool 'data: true'
```
