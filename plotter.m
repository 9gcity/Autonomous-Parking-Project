%Plotting Program

parkspace = ParkingLot(4.8, 2.4, 0, 0);

figure('Name', 'Path Simulation');
plot(xHist(:,1), xHist(:,2), 'b', 'LineWidth', 2.5);
hold on;
plot(xref(:,1), xref(:,2), 'r', 'LineWidth', 2.5);
hold on; 
syms x;
fplot(((heaviside(x - parkspace.backx) * parkspace.backy) - 0.5*parkspace.backy), 'Color', 'magenta');
% hold on;
% plot(xref(:,1), waypoints(:,3));
hold off;
legend({'Nonlinear MPC Simulation', 'Reference Trajectory'}, 'FontSize', 20, 'Location', 'southeast');
xlabel('X-Coordinate (in m)', 'FontSize', 25);
ylabel('Y-Coordinate (in m)', 'FontSize', 25);
grid on;
set(gca, 'FontSize', 20);
axis equal;

figure('Name','Other State Variables');
subplot(2,2,1);
plot(time_arr, xHist(1:(end-1),5), 'LineWidth', 2);
hold on;
plot(time_arr(2:end), xref(1:end-1,4), 'LineWidth', 2);
hold off;
legend({'Nonlinear MPC Simulation', 'Reference Velocity'}, 'FontSize', 15, 'Location', 'north');
xlabel('Time (in seconds)', 'FontSize', 20);
ylabel('Velocity (in m/s)', 'FontSize', 20);
grid on;
set(gca, 'FontSize', 20);

subplot(2,2,2);
plot(time_arr, xHist(1:(end-1), 6), 'LineWidth', 2);
hold on;
plot(time_arr(2:end), xref(1:end-1, 5), 'LineWidth', 2);
hold off;
legend({'Nonlinear MPC Simulation', 'Reference Acceleration'}, 'FontSize', 15, 'Location', 'southeast');
xlabel('Time (in seconds)', 'FontSize', 20);
ylabel('Acceleration (in m/s^2)', 'FontSize', 20);
grid on;
set(gca, 'FontSize', 20);

subplot(2,2,3);
plot(time_arr, xHist(1:(end-1),3), 'LineWidth', 2);
hold on;
plot(time_arr(2:end), xref(1:end-1,3), 'LineWidth', 2);
hold off;
legend({'Nonlinear MPC Simulation', 'Reference Orientation'}, 'FontSize', 15, 'Location', 'south');
xlabel('Time (in seconds)', 'FontSize', 20);
ylabel('Orientation (in radians)', 'FontSize', 20);
grid on;
set(gca, 'FontSize', 20);

subplot(2,2,4);
plot(time_arr, xHist(1:(end-1),4), 'LineWidth', 2);
xlabel('Time (in seconds)', 'FontSize', 20);
ylabel('Steering angle (in radians)', 'FontSize', 20);
grid on;
set(gca, 'FontSize', 20);
