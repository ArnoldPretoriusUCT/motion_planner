% stop robot
pub_wheels_cmd = rospublisher('/duckiebot/wheels_driver_node/wheels_cmd', 'duckietown_msgs/WheelsCmdStamped');
msg_wheels_cmd = rosmessage(pub_wheels_cmd);
msg_wheels_cmd.VelLeft = 0;
msg_wheels_cmd.VelRight = 0;
while 1
    try
        send(pub_wheels_cmd, msg_wheels_cmd);
        break;
    catch
    end
end
