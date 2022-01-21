# drone_competition
for DeltaX competition


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
