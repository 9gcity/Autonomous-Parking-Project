classdef Pose
    properties
        x %X Coordinate of the Vehicle's front axle's centre
        y %Y Coordinate of the Vehicle's front axle's centre
        psi %Angle between vehicle's heading direction and the X-axis
        xdot %Velocity at the current pose
        accx %Acceleration at the current pose
        accy
    end
    
    methods
        function obj = Pose(xin,yin,psin,xdotin,accxin,accyin)
            if nargin == 6
                obj.x = xin;
                obj.y = yin;
                obj.psi = psin;
                obj.xdot = xdotin;
                obj.accx = accxin;
                obj.accy = accyin;
            end
        end
    end
end