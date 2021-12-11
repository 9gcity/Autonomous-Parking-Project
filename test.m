%These values correspond to VOLSKWAGEN POLO 2017 ONWARDS MODEL, 1.0 EVO
%80 ACTIVE 5dr
pathp = Pathplanner;
pathp.turnrad = 5.3;
pathp.whlbase = 2.551;
pathp.axdist = 1.5;
pathp.outer_length = 4.053;
pathp.outer_width = 1.964;
pathp.topspeed = 8; %This is a safe top speed, not the actual top speed of the car
pathp.maxacc = 1.8;
pathp.maxdeacc = 3;
pathp.maxjerk = 0.6;
pathp.maxsteerspeed = deg2rad(20); %Corresponds to 20deg/s
fric = 1;

goal = Pose(0, 0, 0, 0, 0, 0);
start = Pose(10, 10, deg2rad(0), 0, 0, 0);
parkspace = ParkingLot(4.8, 2.4, 0, 0);
waypoints = fifthorderTrack(pathp, start, goal);
[dist, time_arr, vel_prof, acc_prof] = velocityProfilerv5(pathp, waypoints, start, goal, fric); 

for i = 1:200
    x(i) = vel_prof(i)*cos(waypoints(i,2));
    y(i) = vel_prof(i)*sin(waypoints(i,2));
end

plot(x,y);