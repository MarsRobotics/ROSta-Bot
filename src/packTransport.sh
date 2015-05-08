rostopic pub ManualCommand command2ros/ManualCommand 90 90 90 90 90 90 0 0 0 0 0 0 0 false --once
read -p "Press any key to continue... " -n1 -s
rostopic pub ManualCommand command2ros/ManualCommand 0 180 0 180 0 180 0 0 0 0 0 0 0 false --once
