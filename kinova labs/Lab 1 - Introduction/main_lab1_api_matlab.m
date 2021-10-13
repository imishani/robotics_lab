gen3_lite=struct('IP_ADDRESS','192.168.1.10','ID','admin','PASSWORD','admin','SESSION_TIMEOUT',uint32(60000),'CONTROL_TIMEOUT',uint32(2000));
q=[0 0 0 0 0 0];
list_q=[45 0 45 45 0 45
    45 0 0 45 45 45
    45 45 0 90 45 45
    45 45 45 90 0 45];
[~, gen3_lite_handle, ~] = kortexApiMexInterface('CreateRobotApisWrapper', gen3_lite.IP_ADDRESS, gen3_lite.ID, gen3_lite.PASSWORD, gen3_lite.SESSION_TIMEOUT, gen3_lite.CONTROL_TIMEOUT);
[~] = kortexApiMexInterface('ReachJointAngles', gen3_lite_handle,int32(0), 0, 0, q); 
pause(10)
[~,BaseFeedback,~,~] = kortexApiMexInterface('RefreshFeedback',gen3_lite_handle);
tool_pose_cart=BaseFeedback.tool_pose
for i=1:4
    [~] = kortexApiMexInterface('ReachJointAngles', gen3_lite_handle,int32(0), 0, 0, list_q(i,:)); 
    pause(10)
    pause;
end
[~] = kortexApiMexInterface('DestroyRobotApisWrapper', gen3_lite_handle);
