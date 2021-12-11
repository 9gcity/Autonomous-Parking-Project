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
Ts = 0.0474;
p = 20;
nx = 6;
nu = 2;
ny = 5;

nlobj = nlmpc(nx,ny,nu);
nlobj.Ts = Ts;
nlobj.Model.StateFcn = "carstatefunc";
nlobj.Model.OutputFcn = "carstatefuncOUT";
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 4;
%TODO - Need to include Jacobian

%WEIGHTS
nlobj.Weights.OutputVariables = [5 5 5 5 5]; %Q Matrix
nlobj.Weights.ManipulatedVariablesRate = [0.2 0.2]; %R matrix

%Bounds
nlobj.States(4).Min = -deg2rad(45);
nlobj.States(4).Max = deg2rad(45);
nlobj.States(5).Min = -pathp.topspeed;
nlobj.States(5).Max = pathp.topspeed;
nlobj.States(6).Min = -pathp.maxacc;
nlobj.States(6).Max = pathp.maxacc;
% nlobj.MV(1).Min = -pathp.maxsteerspeed;
% nlobj.MV(1).Max = pathp.maxsteerspeed;
nlobj.MV(2).Min = -pathp.maxjerk; %Min jerk
nlobj.MV(2).Max = pathp.maxjerk; %Max jerk

%Reference Setup
goal = Pose(0, 0, 0, 0, 0, 0);
start = Pose(10, 10, deg2rad(0), 0, 0, 0);
parkspace = ParkingLot(4.8, 2.4, 0, 0);
waypoints = fifthorderTrack(pathp, start, goal);
waypoints(:,2) = deg2rad(waypoints(:,2));
[dist, time_arr, vel_prof, acc_prof] = velocityProfilerv5(pathp, waypoints, start, goal, fric); 
vel_prof = -vel_prof;
%acc_prof = -acc_prof;
acc_prof = flip(acc_prof);
xref = [waypoints(2:end, 4), waypoints(2:end, 1), waypoints(2:end, 2), vel_prof(1,2:end)', acc_prof(1,2:end)';
        waypoints(end, 4), waypoints(end, 1), waypoints(end, 2), vel_prof(1,end)', acc_prof(1,end)'];

%Initial Conditions
x0 = [start.x;
      start.y;
      start.psi;
      0; %Zero Initial Steering angle
      0; %Zero Initial Velocity
      0]; %Zero Initial Acceleration
u0 = [0; 0];

%Validating the controller parameters
validateFcns(nlobj,x0,u0);

%Kalman Filter Stuff
Dstatefunc = @(xk,uk,Ts) carstatefuncDiscrete(xk,uk,Ts);
DmeasFcn = @(xk) xk([1:3 5:6]);
EKF = extendedKalmanFilter(Dstatefunc, DmeasFcn, x0);
EKF.MeasurementNoise = 0.01;

%Simulation
Tsteps = 200;
xHist = x0';
uHist = [];
lastMV = zeros(nu,1);
options = nlmpcmoveopt;


for i = 1:Tsteps
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

figure('Name', 'Path Simulation');
grid on;
plot(xHist(:,1), xHist(:,2));
hold on;
plot(xref(:,1), xref(:,2));
hold off;

figure('Name','Velocity Comparison');
plot(xHist(:,5));
hold on;
plot(vel_prof);
hold off;

figure('Name','Acceleration Comparison');
plot(xHist(:,6));
hold on;
plot(acc_prof);
hold off;

figure('Name','Orientation');
plot(xHist(:,3));
hold on;
plot(waypoints(:,2));
hold off;