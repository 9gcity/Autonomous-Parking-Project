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
[dist, time_arr, vel_prof] = velocityProfilerv5(pathp, waypoints, start, goal, fric); 

figure;
plot(waypoints(:,4), waypoints(:,1), '--bo'); %1st Column is y, 4th is x
grid on;
%hold on
%plot(waypoints(:,4), waypoints(:,2), '--ro');
% hold on
% plot(waypoints(:,4), waypoints(:,3), '--go');
%hold off
% title('Parallel Parking Trajectory using 5th Order Polynomial Arc','FontSize',40);
xlabel({'X - Coordinates','(in m)'},'FontSize',35);
ylabel({'Y - Coordinates','(in m)'},'FontSize',35);
%axis square; 
% % axis([0 10 0 10]);
daspect([1 1 1]);
% hold on 
% syms x
% fplot((heaviside(x - parkspace.backx) * parkspace.backy));
% hold off
figure;
plot(time_arr, vel_prof);
grid on;
xlabel({'Time','(in seconds)'}, 'FontSize', 20);
ylabel({'Velocity','(in m/s)'}, 'FontSize', 20);

% figure;
% %time_arr((numel(time_arr) - 2):numel(time_arr)) = [];
% dist((numel(dist) - 2):numel(dist)) = [];
% plot(time_arr', waypoints(:,5)); 
% hold on;
% plot(time_arr', dist);
% hold off;
% grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Operating variables set
%u1 = [1, 3, 5, 7, 9];
u1 = [-9, -7, -5, -3, -1];
x3 = [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5];
x4 = [-37.5, -22.5, -7.5, 7.5, 22.5, 37.5];
C = diag([1,1,1,1]);
D = zeros(4,2);
% C = diag([1, 1, 1, 1, 1]);
% D = zeros(5,2);

%Initial conditions
x0(:,1) = [start.x;
      start.y;
      start.psi;
      0];
xref = [start.x; %Reference X coord
        start.y; %Ref y coord
        start.psi; %Ref orientation
        0]; %Ref steering angle
dt = 0.039;
nx3 = numel(x3);
nx4 = numel(x4);
nu1 = numel(u1);

%u = [vel_prof',
[A,B,K] = linearization_V3(u1, x3, x4, pathp);
%[A, B] = linearization_V2(u1, x3, x4, pathp);
%[A, B, alpha, beta] = linearization_V3(u1, x3, x4, pathp);
%NEED TO PUT THIS IN A LOOP
%x = zeros
Q = diag([0.1,0.1,0.1,0.1]);
%Q = diag([1,1,1,1,1]);
R = diag([0.1,0.1]);
i2 = round(((x0(3)/(x3(nx3) - x3(1))) * nx3) + 1);
i3 = 1;
u_temp = K(:,:,1,i2,i3)*(x0 - xref); 
i1 = round(((u_temp(1)/(u1(5) - u1(1))) * 5) + 1);

for i0 = 1:(numel(time_arr) - 1)
    %NEED TO SET THE A AND B MATRICES ACCORDING TO THE VALUES OF X3 X4 AND
    %U1 
%     A_temp = A(:,:,i1, i2, i3);
%     B_temp = B(:,:,i1, i2, i3);
%     K_temp = K(:,:,1,1,1);
    
%     sys = ss(A_temp, B_temp, C, D);
%     [K, S, e] = lqr(sys, Q, R);
    
    dt = time_arr(i0 + 1) - time_arr(i0);
%     u_temp = K(:,:,i1,i2,i3)*(x0 - xref);
    xdot1 = u_temp(1)*cos(x0(3));
    xdot2 = u_temp(1)*sin(x0(3));
    xdot3 = (u_temp(1)*tan(x0(4)))/pathp.whlbase;
    xdot4 = u_temp(2);
    x0(:, i0+1) = [x0(1,i0) + xdot1*dt;
                   x0(2,i0) + xdot2*dt;
                   x0(3,i0) + xdot3*dt;
                   x0(4,i0) + xdot4*dt];
%     t = [time_arr(i0), time_arr(i0+1)];
%     sys = ss((A_temp - B_temp*K), B_temp, C, D);
%     [y, ~, x] = lsim(sys, u, t, x0);
%     figure;
%     plot(y(1),t);
    
    A = [0, 0, (-u_temp(1)*sin(x0(3,i0))), 0;
         0, 0, (u_temp(1)*cos(x0(3,i0))), 0;
         0, 0, 0, (u_temp(1)*(1/pathp.whlbase)*(sec(x0(4,i0))^2));
         0, 0, 0, 0];
                      
    B = [cos(x0(3,i0)), 0;
         sin(x0(3,i0)), 0;
         (1/pathp.whlbase)*tan(x0(3,i0)), 0;
         0, 1];
    sys = ss(A, B, C, D);
    [K,~,~] = lqr(sys, Q,R);
    
    xref = [waypoints(i0 + 1,4); %Reference X coord
            waypoints(i0 + 1,1); %Ref y coord
            waypoints(i0 + 1,2); %Ref orientation
            pathp.whlbase/waypoints(i0 + 1,3)]; %Ref steering angle
    u_temp = K*(xref - x0(:, i0));
end
