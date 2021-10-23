classdef ParkingLot %To hold the parking lot dimensions etc
    properties
        backy %Y-Coord of the upper back corner of the parking space
        backx %X-Coord ''
        length 
        depth
    end
    
    methods
        function obj = ParkingLot(backxin, backyin, lengthin, depthin)
            if nargin == 4
                obj.backx = backxin; %X-Coordinate of the back point of the parking lot
                obj.backy = backyin; %Y-Coordinate of the back point of the parking lot
                obj.length = lengthin; %length of the lot
                obj.depth = depthin; %Depth of the lot
            end
        end
    end
end
