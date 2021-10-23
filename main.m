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
start = Pose(15, 5, deg2rad(0), 0, 0, 0);
parkspace = ParkingLot(4.8, 2.4, 0, 0);
waypoints = fifthorderTrack(pathp, start, goal);
[time_arr, vel_prof] = velocityProfilerv5(pathp, waypoints, start, goal, fric); 
plot(waypoints(:,4), waypoints(:,1), '--bo'); %1st Column is y, 4th is x
%hold on
%plot(waypoints(:,4), waypoints(:,2), '--ro');
% hold on
% plot(waypoints(:,4), waypoints(:,3), '--go');
%hold off
% title('Parallel Parking Trajectory using 5th Order Polynomial Arc','FontSize',40);
% xlabel({'X - Coordinates','(in m)'},'FontSize',35);
% ylabel({'Y - Coordinates','(in m)'},'FontSize',35);
%axis square; 
% % axis([0 10 0 10]);
daspect([1 1 1]);
% hold on 
% syms x
% fplot((heaviside(x - parkspace.backx) * parkspace.backy));
% hold off
figure;
plot(time_arr, vel_prof);
xlabel({'Time','(in seconds)'}, 'FontSize', 20);
ylabel({'Velocity','(in m/s)'}, 'FontSize', 20);
