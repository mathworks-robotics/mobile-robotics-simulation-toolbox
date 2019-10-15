classdef RobotDetector < matlab.System & matlab.system.mixin.CustomIcon & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime
    % ROBOTDETECTOR Robot detector simulator
    %
    % Returns the angles, ranges, and labels of a simulated robot detector
    %
    % For more information, see <a href="matlab:edit mrsDocRobotDetector">the documentation page</a>
    %
    % Copyright 2018-2019 The MathWorks, Inc.
    
    %% PROPERTIES
    % Public (user-visible) properties
    properties
        robotIdx = 1;           % Robot index
        sensorOffset = [0,0];   % Robot detector offset (x,y) [m]
        sensorAngle = 0;        % Robot detector angle [rad]
        fieldOfView = pi/3;     % Sensor field of view [rad]
        maxRange = 5;           % Maximum range [m]
        maxDetections = 3;      % Maximum number of detections
    end
    
    properties(Nontunable)
       sampleTime = 0.1; % Sample time
    end

    % Private properties
    properties(Access = private)
        env;        % MultiRobotEnv object
        map;        % Occupancy grid
        hasMap;     % Binary flag for having a map
    end

    %% METHODS    
    methods
        % Constructor: Optionally accepts the following arguments:
        % 1: MultiRobotEnv object (defaults to global variable for Simulink)
        % 2: Robot index (defaults to 1)
        function obj = RobotDetector(varargin)
            if nargin > 1
               obj.robotIdx = varargin{2}; 
            else
               obj.robotIdx = 1;
            end
            if nargin > 0 
                obj.env = varargin{1};
            else
                % Get to this step only if the constructor is called
                % without arguments. Defaults to behavior compatible with
                % Simulink blocks.
                try
                    global slMultiRobotEnv  
                    obj.env = slMultiRobotEnv;
                catch
                    error('No Multi-Robot Environment available');
                end
            end
            
        end
    end
    
    methods(Access = protected)
      
        function setupImpl(obj)           
            
           if isempty(obj.env)
                global slMultiRobotEnv
                obj.env = slMultiRobotEnv;
           end
           
           % Attach the robot detector to the environment
           attachRobotDetector(obj.env,obj);
           
           % Ensure to use the same map as the visualizer
           obj.map = internal.createMapFromName(obj.env.mapName);
           obj.hasMap = ~isempty(obj.map);
           
        end
        
        % Step method: Wraps around the step method of the RobotDetector
        function detections = stepImpl(obj)                 
            % Initialize
            ranges = [];
            angles = [];
            labels = [];
            
            % Find the sensor pose and check if valid
            pose = obj.env.Poses(:,obj.robotIdx);
            theta = pose(3);
            offsetVec = [cos(theta) -sin(theta); ...
                         sin(theta)  cos(theta)]*obj.sensorOffset';
            sensorPose = pose + [offsetVec; obj.sensorAngle];  
            if ~obj.hasMap
                validPose = true;
            else
                validPose = sensorPose(1) >= obj.map.XWorldLimits(1) && ...
                            sensorPose(1) <= obj.map.XWorldLimits(2) && ... 
                            sensorPose(2) >= obj.map.YWorldLimits(1) && ... 
                            sensorPose(2) <= obj.map.YWorldLimits(2);
            end
            
            % Return the range and angle for all robots
            % First, find the offsets
            offsets = obj.env.Poses(1:2,:) - sensorPose(1:2);
            offsets(:,obj.robotIdx) = inf; % Take out the "self-offset"
            
            if ~isempty(offsets) && validPose
                % Extract ranges and angles
                ranges = sqrt(sum(offsets.^2,1));
                angles = wrapToPi(atan2(offsets(2,:),offsets(1,:))-sensorPose(3));
                
                % Filter by maximum range and field of vision
                validIdx = (ranges <= obj.maxRange) & ... 
                           (abs(angles) <= obj.fieldOfView/2);
                ranges = ranges(validIdx);
                angles = angles(validIdx);
                labels = find(validIdx);
                                
                % Use occupancy grid, if any, to account for obstacles
                if ~isempty(obj.map)
                    % Loop backwards since we're removing values
                    for idx = numel(ranges):-1:1  
                        intPts = rayIntersection(obj.map,sensorPose, ... 
                                                 angles(idx),ranges(idx));
                        % Delete the reading if the point is occupied
                        if ~isnan(intPts)
                            ranges(idx) = [];
                            angles(idx) = [];
                            labels(idx) = [];
                        end
                    end
                end
                
            end
            
            if ~isempty(ranges)
                % Sort from nearest
                [ranges,sortedIdx] = sort(ranges);
                angles = angles(sortedIdx);
                labels = labels(sortedIdx);
                if numel(ranges) > obj.maxDetections
                    ranges = ranges(1:obj.maxDetections);
                    angles = angles(1:obj.maxDetections);
                    labels = labels(1:obj.maxDetections);
                end
                
                % Pack the final results into the output
                detections = [ranges', angles', labels'];
            else
                detections = []; 
            end
                    
        end

        % More methods needed for the Simulink block to inherit its output
        % sizes from the scan angle parameter provided.
        function sz = getOutputSizeImpl(obj)
            sz = [obj.maxDetections,3];
        end
        
        function fx = isOutputFixedSizeImpl(~)
           fx = false;
        end
        
        function dt = getOutputDataTypeImpl(~)
            dt = 'double';
        end

        function cp = isOutputComplexImpl(~)
            cp = false;
        end
        
        % Define icon for System block
        function icon = getIconImpl(obj)
            icon = {['Robot ' num2str(obj.robotIdx)],'Detector'};
        end
        
        % Define sample time
        % Must be discrete since the output is a variable-sized signal
        function sts = getSampleTimeImpl(obj)
            if obj.sampleTime > 0
               sts = createSampleTime(obj,'Type','Discrete','SampleTime',obj.sampleTime);
            else
               sts = createSampleTime(obj,'Type','Inherited'); 
            end
        end
        
        % Save and load object methods to enable stepping
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            s.env = obj.env;
            s.map = obj.map;
        end
        function loadObjectImpl(obj,s,wasInUse)
            obj.map = s.map;
            obj.env = s.env;
            loadObjectImpl@matlab.System(obj,s,wasInUse);
        end
                
    end
    
    methods (Static, Access = protected)
        % Do not show "Simulate using" option
        function flag = showSimulateUsingImpl
            flag = false;
        end
        % Always run in interpreted mode
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end
    end
    
    
    
end
