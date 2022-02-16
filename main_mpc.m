%These values correspond to VOLSKWAGEN POLO 2017 ONWARDS MODEL, 1.0 EVO
%80 ACTIVE 5dr
pathp = Pathplanner;
pathp.turnrad = 5.3;
pathp.whlbase = 2.551;
pathp.axdist = 1.5;
pathp.outer_length = 4.053;
pathp.outer_width = 1.964;
pathp.topspeed = 10; %This is a safe top speed, not the actual top speed of the car
pathp.maxacc = 1.8;
pathp.maxdeacc = 3;
pathp.maxjerk = 0.6;
pathp.maxsteerspeed = deg2rad(20); %Corresponds to 20deg/s
fric = 1;

%Controller Parameters
nx = 6;
nu = 2;
ny = 5;
Tsteps = 200;

%Reference Setup
goal = Pose(1, 1, 0, 0, 0, 0); 
start = Pose(15, 5, deg2rad(0), 0, 0, 0);
parkspace = ParkingLot(4.8, 3, 0, 0);
waypoints = fifthorderTrack_V3(pathp, start, goal, parkspace, Tsteps);
waypoints(:,2) = deg2rad(waypoints(:,2));
[dist, time_arr, vel_prof, acc_prof] = velocityProfilerv5(pathp, waypoints, start, goal, fric); 
vel_prof = -vel_prof;
acc_prof = flip(acc_prof);
xref = [waypoints(2:end, 4), waypoints(2:end, 1), waypoints(2:end, 2), vel_prof(1,2:end)', acc_prof(1,2:end)';
        waypoints(end, 4), waypoints(end, 1), waypoints(end, 2), vel_prof(1,end)', acc_prof(1,end)'];

Ts = (time_arr(end) - time_arr(1))/Tsteps;

nlobj = nlmpc(nx,ny,nu);
nlobj.Ts = Ts;
nlobj.Model.StateFcn = "carstatefunc";
nlobj.Model.OutputFcn = "carstatefuncOUT";
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 4;

%WEIGHTS
nlobj.Weights.OutputVariables = 5 * [1 1 1 1 1]; %Q Matrix
nlobj.Weights.ManipulatedVariablesRate = 0.1 * [1 1]; %R matrix

%Bounds
nlobj.States(4).Min = -deg2rad(45);
nlobj.States(4).Max = deg2rad(45);
nlobj.OV(4).Min = -pathp.topspeed;
nlobj.OV(4).Max = pathp.topspeed;
nlobj.OV(5).Min = -10;%pathp.maxacc;
nlobj.OV(5).Max = pathp.maxacc;
nlobj.MV(2).Min = -pathp.maxjerk; %Min jerk
nlobj.MV(2).Max = pathp.maxjerk; %Max jerk

%Initial Conditions
x0 = [start.x;
      start.y;
      start.psi;
      0; %Zero Initial Steering angle
      0; %Zero Initial Velocity
      0]; %Zero Initial Acceleration
u0 = [0; 0];

%Validating the controller parameters - UNCOMMENT THE LINE BELOW TO
%VALIDATE THE VARIOUS PARAMETERS OF THE NONLINEAR MPC CONTROLLER
% validateFcns(nlobj,x0,u0);

%Kalman Filter Stuff
Dstatefunc = @(xk,uk,Ts) carstatefuncDiscrete(xk,uk,Ts);
DmeasFcn = @(xk) xk([1:3 5:6]);
EKF = extendedKalmanFilter(Dstatefunc, DmeasFcn, x0);
EKF.MeasurementNoise = 0.01;

%Simulation
xHist = x0';
uHist = [];
lastMV = zeros(nu,1);
options = nlmpcmoveopt;

%Simulation Loop
tic;
for i = 1:Tsteps
%%Terminal State Overshoot Checker - Uncomment below lines to use
%     if xHist(i,1) < goal.x || xHist(i,2) < goal.y
%         break
%     end
    yk = xHist(i,[1:3 5:6])' + randn*0.01;
    xk = correct(EKF, yk);
    [uk, options] = nlmpcmove(nlobj, xk, lastMV, xref(i:min(i+9, Tsteps),:), [], options);
    predict(EKF, uk, Ts);
    uHist(i,:) = uk';
    lastMV = uk;
    ODEFUNC = @(t,xk) carstatefunc(xk,uk);
    [TOUT,YOUT]  = ode45(ODEFUNC,[0 Ts], xHist(i,:)');
    xHist(i+1,:) = YOUT(end,:);
end
time = toc;
time_1 = toc/200;

%Plotting Information
figure('Name', 'Path Simulation');
plot(xHist(:,1), xHist(:,2), 'b--o');
hold on;
plot(xref(:,1), xref(:,2), 'r--o');
hold off;
% title('Path Simulation', 'FontSize', 30);
grid on;
 
% figure('Name','Velocity Comparison');
% plot(time_arr, xHist(1:(end-1),5));
% hold on;
% plot(time_arr, xref(:,4));
% hold off;
% % title('Velocity Comparison', 'FontSize', 30);
% grid on;
% 
% figure('Name','Acceleration Comparison');
% plot(time_arr, xHist(1:(end-1), 6));
% hold on;
% plot(time_arr, xref(:, 5));
% hold off;
% % title('Acceleration Comparison', 'FontSize', 30);
% grid on;
% 
% figure('Name','Orientation');
% plot(time_arr, xHist(1:(end-1),3));
% hold on;
% plot(time_arr, xref(:,3));
% hold off;
% % title('Orientation Comparison', 'FontSize', 30);
% grid on;
% 
% figure('Name','Steering Angle');
% plot(time_arr, xHist(1:(end-1),4));
% % title('Steering Angle', 'FontSize', 30);
% grid on;