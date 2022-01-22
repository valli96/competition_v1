# Delta X drone competition

## Next task

- [x] fix camera rotation
- [x] look that the transformation tree exist
- [x] add new marker script for the competition marker 
- [x] implement **state 8**
- [ ] try to run the code
- [ ] tune PID controller for height
- [ ] check if stage 0 to 2 works fine
      - [ ] 3 successful following
      - [ ] find good height for the upper marker



- [ ] PID controller for rotation 
- [ ] implement the landing proses **state 5** 
      - [ ] tune the PID controller for this stage (try to uses velocity) 
      - [ ] use different drones
      - [ ] find the write height
      - [ ] have 3 successfully on the marker 
      - [ ] have 3 successful test on the ground vehicle


- [ ] read drone_pos from opto-track
- [ ] implement velocity function based on 2 sources **function 1 in .svg**
- [ ] display the velocity calculated before in the graph and display it
- [ ] implement **function 2**
- [ ] implement detection of marker stopt 
- [ ] combine following and landing by implementing  **state 3**
- [ ] change **state 3** based on some feedback (now without feedback) 
    - use msg type 
    - create a new global variable at the moment the marker is not visible any more
    - PID controller removes input(last_dist_marker_x(from csv) - msg.pose_differance)  
    - 
- [ ] adding the second marker to the function
- [ ] change **state 2** to handel 2 markers
- [ ] implement different error handlers for obivous wrong values
- [ ] use the start estimation of the ground vehicle to improve the certainty of the ground vehicle position

### nice to have
- [ ] implement **state 7**
- [ ] change into main into a class
- [ ] remove all spelling errors


## Use opto-track 

1. start the PC
2. start motive
3. start steaming under view 
	- Local inervace: 192.168.1.234
	- Up Axis:  Z up
4. install vrpn_client_ros (can be find at the ros wiki)
5. connect to the network with a Lan cabel (fist the range needed to be defined)
5. `roslauch vrpn_client_ros sample.launch server:=192.168.1.234`
6. rostopic list 
7. rostopic echo \*/drone_1 & \*/jackal
8. this this should be a pose msg


### how to rotate camera


### open transformation tree
