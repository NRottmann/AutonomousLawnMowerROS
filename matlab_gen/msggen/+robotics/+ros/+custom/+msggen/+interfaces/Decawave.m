classdef Decawave < robotics.ros.Message
    %Decawave MATLAB implementation of interfaces/Decawave
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'interfaces/Decawave' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'b2acc40fd1a640d4b9a410d1bbfe92ac' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        StdMsgsHeaderClass = robotics.ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        X
        Y
        Z
        Confidence
    end
    
    properties (Access = protected)
        Cache = struct('Header', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Confidence', 'Header', 'X', 'Y', 'Z'} % List of non-constant message properties
        ROSPropertyList = {'confidence', 'header', 'x', 'y', 'z'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = Decawave(msg)
            %Decawave Construct the message object Decawave
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'Decawave', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function x = get.X(obj)
            %get.X Get the value for property X
            x = single(obj.JavaMessage.getX);
        end
        
        function set.X(obj, x)
            %set.X Set the value for property X
            validateattributes(x, {'numeric'}, {'nonempty', 'scalar'}, 'Decawave', 'X');
            
            obj.JavaMessage.setX(x);
        end
        
        function y = get.Y(obj)
            %get.Y Get the value for property Y
            y = single(obj.JavaMessage.getY);
        end
        
        function set.Y(obj, y)
            %set.Y Set the value for property Y
            validateattributes(y, {'numeric'}, {'nonempty', 'scalar'}, 'Decawave', 'Y');
            
            obj.JavaMessage.setY(y);
        end
        
        function z = get.Z(obj)
            %get.Z Get the value for property Z
            z = single(obj.JavaMessage.getZ);
        end
        
        function set.Z(obj, z)
            %set.Z Set the value for property Z
            validateattributes(z, {'numeric'}, {'nonempty', 'scalar'}, 'Decawave', 'Z');
            
            obj.JavaMessage.setZ(z);
        end
        
        function confidence = get.Confidence(obj)
            %get.Confidence Get the value for property Confidence
            confidence = single(obj.JavaMessage.getConfidence);
        end
        
        function set.Confidence(obj, confidence)
            %set.Confidence Set the value for property Confidence
            validateattributes(confidence, {'numeric'}, {'nonempty', 'scalar'}, 'Decawave', 'Confidence');
            
            obj.JavaMessage.setConfidence(confidence);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.X = obj.X;
            cpObj.Y = obj.Y;
            cpObj.Z = obj.Z;
            cpObj.Confidence = obj.Confidence;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.X = strObj.X;
            obj.Y = strObj.Y;
            obj.Z = strObj.Z;
            obj.Confidence = strObj.Confidence;
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.X = obj.X;
            strObj.Y = obj.Y;
            strObj.Z = obj.Z;
            strObj.Confidence = obj.Confidence;
            strObj.Header = saveobj(obj.Header);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.interfaces.Decawave.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.interfaces.Decawave;
            obj.reload(strObj);
        end
    end
end
