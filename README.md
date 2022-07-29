# uno_payload
ROS Noetic files for payload control via Rosserial Arduino.

# Setup
Ensure that the Arduino is uploaded with the Arduino script, and connected to the router with the main ROS computer. 

Check the IP address of the router and ROS computer and edit lines 25 to 31 in `uno_payload_controller_mkrzero.ino`.

```
// Shield settings
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x28, 0xCE };
IPAddress ip(192, 168, 0, 177);

// Server settings
IPAddress server(192, 168, 0, 10);
uint16_t serverPort = 11411;
```

The IP address of the server is the IP address of the ROS computer and the IP address of the shield can be any unused IP.

# Launch
``` 
roslaunch rosserial_server socket.launch
rosrun payload_action payload_action_server
```

# Calling the Action
```
PayLoadMode pusher_action
---
PayLoadMode pusher_state
bool success
---
PayLoadMode pusher_state
bool success
```

The payload can be controlled to the deposit or collection state by sending the action goal,
- 0: Deposit or up state
- 1: Collect or down state
- 2: Idle state (usually not used)
