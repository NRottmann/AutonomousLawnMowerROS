classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    properties (Constant)
        interfaces_Bumper = 'interfaces/Bumper'
        interfaces_Capacity = 'interfaces/Capacity'
        interfaces_Chlorophyll = 'interfaces/Chlorophyll'
        interfaces_Control = 'interfaces/Control'
        interfaces_Decawave = 'interfaces/Decawave'
        interfaces_IMU = 'interfaces/IMU'
        interfaces_Odometry = 'interfaces/Odometry'
        interfaces_Radar = 'interfaces/Radar'
        interfaces_Sensor = 'interfaces/Sensor'
        localization_Pose = 'localization/Pose'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(10, 1);
                msgList{1} = 'interfaces/Bumper';
                msgList{2} = 'interfaces/Capacity';
                msgList{3} = 'interfaces/Chlorophyll';
                msgList{4} = 'interfaces/Control';
                msgList{5} = 'interfaces/Decawave';
                msgList{6} = 'interfaces/IMU';
                msgList{7} = 'interfaces/Odometry';
                msgList{8} = 'interfaces/Radar';
                msgList{9} = 'interfaces/Sensor';
                msgList{10} = 'localization/Pose';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
