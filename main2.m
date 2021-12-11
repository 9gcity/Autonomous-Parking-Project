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
vel_prof = -vel_prof;

x5 = [-10, -8, -6, -4, -2, 2, 4, 6, 8, 10];
x3 = [-157.5, -112.5, -67.5, -22.5, 22.5, 67.5, 112.5, 157.5];
x3 = deg2rad(x3);
x4 = [-37.5, -22.5, -7.5, 7.5, 22.5, 37.5];
x4 = deg2rad(x4);

% C = diag([1, 1, 1, 1, 1]);
% D = zeros(5,2);

%Initial conditions
x0(:,1) = [start.x;
          start.y;
          start.psi;
          0;
          start.xdot;
          0;
          0;
          0];
% xref = [waypoints(1,4); %Reference X coord
%         waypoints(1,1); %Ref y coord
%         waypoints(1,2); %Ref orientation
%         0; %Initial ref steering angle
%         vel_prof(1);
%         0;
%         0;
%         0]; %Ref velocity
xref = [0;
        0;
        0;
        0;
        0;
        waypoints(1,4);
        waypoints(1,1);
        waypoints(1,2)];
    
dt = 0.0474;
nx3 = numel(x3);
nx4 = numel(x4);
nx5 = numel(x5);

Q = diag([1,1,1,1,1]);
R = diag([1,1]);
[A,B,Kd1,Kd2,Kx,const] = linearization_V6(x3, x4, x5, pathp, Q, R);
i2 = recursive_search(x0(3,1), x3);
i2 = find(x3 == i2);

i3 = 1;
u_temp = K(:,:,5,i2,i3)*(xref - x0); %%WHY DOES THIS RESULT IN A 3X2 ARRAY
%i1 = round((((x0(5))/(x5(nx5) - x5(1))) * nx5) + 1); %POINT OF CONTENTION
i1 = recursive_search(x0(5,1), x5);
i1 = find(x5 == i1);
% time_arr = linspace(0,50,200);
% dt = time_arr(2) - time_arr(1);
for i0 = 1:(numel(time_arr) - 2)
    %NEED TO SET THE A AND B MATRICES ACCORDING TO THE VALUES OF X3 X4 AND
    %U1 
    
%     dt = time_arr(i0 + 1) - time_arr(i0);
%     u_temp = K(:,:,i1,i2,i3)*(x0 - xref);
    xref = [waypoints(i0+1,4);
            waypoints(i0+1,1);
            waypoints(i0+1,2);
            pathp.whlbase/(waypoints(i0+1,3));
            vel_prof(i0+1);
            ];
    xdot1 = x0(5, i0)*cos(x0(3, i0));
    xdot2 = x0(5, i0)*sin(x0(3, i0)); 
    xdot3 = (x0(5, i0)*tan(x0(4, i0)))/pathp.whlbase; 
    xdot4 = u_temp(1);
    xdot5 = x0(6,i0);
    xdot6 = u_temp(2);
    
    x0(:, i0+1) = [x0(1, i0) + xdot1*dt;
                   x0(2, i0) + xdot2*dt;
                   x0(3, i0) + xdot3*dt;
                   x0(4, i0) + xdot4*dt;
                   x0(5, i0) + xdot5*dt;
                   x0(6, i0) + xdot6*dt];
    
    i1 = recursive_search(x0(5,i0), x5);
    i1 = find(x5 == i1);
    
    i2 = recursive_search(x0(3,i0), x3);
    i2 = find(x3 == i2);
    
    i3 = recursive_search(x0(4,i0), x4);
    i3 = find(x4 == i3);
    
%     xref = [waypoints(i0+1,4); %Reference X coord
%             waypoints(i0+1,1); %Ref y coord
%             waypoints(i0+1,2); %Ref orientation
%             pathp.whlbase/waypoints(i0+1,3); %Ref steering angle
%             vel_prof(i0 + 1)]; %Ref velocity
    u_temp = K(:,:,i1,i2,i3)*(x0);
end

figure('Name', 'Comparison between the reference and the actual path');
plot(waypoints(:, 4), waypoints(:, 1), 'b');
hold on;
plot(x0(1,:), x0(2,:), 'r');
% hold on;
% plot(waypoints(:,4), dist);
hold off;

figure('Name', 'Comparison between the reference and the actual velocity');
%plot(time_arr, vel_prof, time_arr, x0(5,:));
plot(vel_prof);
hold on;
plot(x0(5,:));
hold off;
