# drone_catching_mpc
Model Predictive Control of a UAV for catching anti drone.

# antidrone
alias start='tmuxp load gazebo_simulation.yaml'
alias target='rosservice call /uav2/mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'" && 
echo =========== &&
rosservice call /uav2/mavros/cmd/arming "value: true"'
alias track='rosservice call /uav1/mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'" && 
echo =========== &&
rosservice call /uav1/mavros/cmd/arming "value: true"'
