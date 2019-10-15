function multiRobotVizSet(idx,pose,ranges)

    % Get the multi-robot environment variable (defined in the Multi-Robot Environment block)
    global slMultiRobotEnv
    
    % Error check
    if isempty(slMultiRobotEnv)
        error('Environment not found. Please add a Multi-Robot Environment block to your model.');
    end
    
    % Visualize the robot
    drawRobots(slMultiRobotEnv,idx,pose,ranges);
    
end