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
pathp.maxjerk = 0.9;
pathp.maxsteerspeed = deg2rad(20); %Corresponds to 20deg/s
fric = 1;

goal = Pose(0, 0, 0, 0, 0, 0);
start = Pose(9, 5, deg2rad(10), 0, 0, 0);
parkspace = ParkingLot(4.8, 2.4, 0, 0);
[vel_prof, waypoints] = fifthorderTrack(pathp, start, goal, fric); 
%plot(waypoints(:,4), waypoints(:,1), '--bo');%,waypoints2(:,1), waypoints2(:,2), '--ro'); %1st Column is x, 2nd is y
% hold on
%plot(waypoints(:,4), waypoints(:,2), '--ro');
% hold on
%plot(waypoints(:,4), waypoints(:,3), '--go');
plot(vel_prof);
% hold off
%title('Parallel Parking Trajectory','FontSize',40);
xlabel({'X - Coordinates','(in m)'},'FontSize',35);
%ylabel({'Y - Coordinates','(in m)'},'FontSize',35);
%ylabel('Slope','FontSize',35);
ylabel({'Curvature','(in m^-1)'},'FontSize',35);
axis square;
%daspect([1 1 1]);
% hold on 
% syms x
% fplot((heaviside(x - parkspace.backx) * parkspace.backy));
% hold off
