%%%TODO - FIX THE VELOCITY PLANNER TO ENSURE CONSISTENT SAMPLE TIMES
classdef Pathplanner
    properties
        whlbase %Width of the wheelbase of the vehicle
        axdist %distance between the two wheels of the vehicle
        outer_length
        outer_width
        turnrad %Turning radius of the vehicle
        topspeed %Safe top speed of the vehicle
        maxacc %Maximum acceleration of the vehicle
        maxjerk
        maxsteerspeed
        maxdeacc
    end
    
    methods
        function waypoints = circleTrack(obj, start, goal, lot)
        %For now this function assumes that the goal pose is closer to the
        %origin than the start pose, and both are situated in the first
        %cartesian quadrant
            arguments
                obj
                start Pose
                goal Pose
                lot ParkingLot
            end
            
            c1 = [goal.x, (goal.y + obj.turnrad)];
            N = 100; %Number of points on the curves
            
            %This calculates the first minimum radius circle arc
            %NEED TO MODIFY SO THAT WHEN THETA_DIS>90, IT MAKES A STRAIGHT
            %LINE
            %theta = acos(((obj.turnrad + goal.y) - lot.backy)/obj.turnrad);
            
            %Using the equation of a circle \/
            goal_mid_dist_x = sqrt((obj.turnrad^2) - (lot.backy - c1(2))^2) - c1(1);
            goal_mid_dist = sqrt(((goal_mid_dist_x - goal.x)^2) + (lot.backy - goal.y)^2);
            theta = acos((2*(obj.turnrad^2) - goal_mid_dist^2)/(2*(obj.turnrad^2)));
            theta_dis = linspace((-(pi/2)), (theta - (pi/2)), N);
            %theta_dis = linspace(pi,-pi);
            arc1 = [obj.turnrad*cos(theta_dis);obj.turnrad*sin(theta_dis)];
            waypoints = transpose(arc1);
            waypoints(:,1) = waypoints(:,1) + c1(1);
            waypoints(:,2) = waypoints(:,2) + c1(2);
            waypoints(:,3) = obj.turnrad; %For use by the velocity profiler
            
            %This calculates the second circle arc
            dce = sqrt(((start.x - c1(1))^2)+((start.y - c1(2))^2));
            alpha = start.psi + acos((start.y - c1(2))/dce);
            REinit = ((dce^2) - (obj.turnrad^2))/(2*obj.turnrad + 2*dce*cos(alpha));
            
            syms x y
            eqns = [((start.x - x)^2) + ((start.y - y)^2) == REinit^2,
                    ((goal_mid_dist_x - x)^2) + ((lot.backy - y)^2) == REinit^2];
                    %((x - c1(1))^2) + ((y - c1(2))^2) == (REinit + obj.turnrad)^2,
                    %(-1)/((lot.backy - c1(2))/(goal_mid_dist_x - c1(1))) == (-1)/((lot.backy - y)/(goal_mid_dist_x - x))];
            cr = solve(eqns,[x y]);
            
            %cr_angle = acos(((REinit^2) + ((obj.turnrad + REinit)^2) - (dce^2)) / (2 * REinit *(obj.turnrad + REinit)));
            start_mid_dist = sqrt(((goal_mid_dist_x - start.x)^2) + (lot.backy - start.y)^2);
            cr_angle = acos((2*(REinit^2) - start_mid_dist^2)/(2 * (REinit^2)));
            cr_angle_dis = linspace((start.psi + (pi/2)), (start.psi + (pi/2) + cr_angle), N);
%             cr_angle_dis = linspace(0, 2*pi, 1000);
            arc2 = zeros(N,3);
            counter = 1;
            for i = 1:N
                if (REinit * sin(cr_angle_dis(i)) + cr.y(2)) >= waypoints(N,2)
                    arc2(counter,1) = REinit * cos(cr_angle_dis(i)) + cr.x(2);
                    arc2(counter,2) = REinit * sin(cr_angle_dis(i)) + cr.y(2);
                    counter = counter + 1;
                end
            end
          
            arc2(:,3) = REinit; %For use by the velocity profiler
            waypoints = [waypoints; arc2];
            %waypoints = flip(waypoints);
        end
        
        function waypoints = clothoidTrack(obj, start, goal, lot)
        %For now this function assumes that the goal pose is closer to the
        %origin than the start pose, and both are situated in the first
        %cartesian quadrant
            arguments
                obj
                start Pose
                goal Pose
                lot ParkingLot
            end
            N = 500; %Number of points on the curves
            maxsteerangle = atan(obj.whlbase / obj.turnrad); %Maximum steering angle of the vehicle
            tmin = maxsteerangle/obj.maxsteerspeed; %Time taken to steer from 0 to max angle
            Lmin = obj.topspeed*tmin; %Arc travelled in that time
            Amin = sqrt(obj.turnrad*Lmin); %Parameter of clothoid
            
            %Coordinates of the point where the clothoid has curvature =
            % 1 / minimum turning radius of the car
            xrmin = Amin * sqrt(pi) * fresnelc(Lmin/(pi*obj.turnrad)); 
            yrmin = Amin * sqrt(pi) * fresnels(Lmin/(pi*obj.turnrad));
            psimin = (Amin^2)/(2*obj.turnrad^2);
            
            %Coordinates of the centre of the clothoid 
            xc = xrmin - obj.turnrad*sin(psimin) + goal.x;
            yc = yrmin + obj.turnrad*cos(psimin) + goal.y;
            %Starting minimum radius of the clothoid
            R1 = sqrt((xc^2) + (yc^2))
            obj.turnrad
            nu = rad2deg(atan(xc/yc))
            path_lengths = linspace(0, Lmin, N);
            
            %Clothoid arc formation
            waypoints(:,1) = Amin * sqrt(pi) * fresnelc(path_lengths/(pi*obj.turnrad)) + goal.x;
            waypoints(:,2) = Amin * sqrt(pi) * fresnels(path_lengths/(pi*obj.turnrad)) + goal.y;
            
            %rmin = sqrt(((xc - lastpt_x)^2) + (yc - lastpt_y)^2); %Should be Rmin, which it is
            R = zeros(500,1);
            for i = 1:500
                %This shows that the last radius of curvature is indeed 5.3
                R(i) = sqrt(((waypoints(i,1) - xc)^2) + ((waypoints(i,2) - yc)^2)); 
            end
            plot(R);
            i = 1;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %theta = acos(((obj.turnrad + goal.y) - lot.backy)/obj.turnrad);
            %triangle_point_A = [
            %triangle_point_B = [goal.x (goal.y + (R1 - obj.turnrad)*cos(nu))];
%             distance_goalpark = sqrt(((goal.x - lot.backx)^2) + (goal.y - lot.backy)^2);
%             theta = (2*(R1^2) - distance_goalpark^2)/(2*(R1^2));
%             theta_dis = linspace((theta - (pi/2)), (-(pi/2)), N);
%             last = numel(waypoints(:,1));
%             counter = 1;
            %theta_dis = linspace(pi,-pi);
%             for i = 1:numel(theta_dis)
%                 arc1(i,1) = (obj.turnrad*cos(theta_dis(i)) + xc); 
%                 arc1(i,2) = (obj.turnrad*sin(theta_dis(i)) + yc);
%                 if arc1(i,1) >= lastpt_x && arc1(i,2) >= lastpt_y
%                     waypoints(last+counter,1) = arc1(i,1);
%                     waypoints(last+counter,2) = arc1(i,2);
%                     counter = counter + 1;
%                 end
%             end

            %Creating a circle here to check the clothoid
            circ_ang = 0:0.01:2*pi;
%             plot(arc1(:,1), arc1(:,2));
            arc1 = zeros(numel(circ_ang), 2);
            arc1(:,1) = (obj.turnrad*cos(circ_ang) + xc);
            arc1(:,2) = (obj.turnrad*sin(circ_ang) + yc);
            waypoints = [waypoints;xc yc];
            waypoints = [waypoints; arc1];
            i = 1; %This is for debugging to set breakpoints
            %waypoints_temp = transpose(arc1);
%             waypoints_temp(:,1) = waypoints_temp(:,1) + xc;%c1(1);
%             waypoints_temp(:,2) = waypoints_temp(:,2) + yc;%c1(2);
%             waypoints_temp(:,3) = obj.turnrad; %For use by the velocity profiler
            %%%%%%%%%%%%%%%%%%
%             tmin = (atan(obj.whlbase/obj.turnrad))/obj.maxsteerspeed;
        end
% TO BE DELETED - OUTDATED
%        function vel_profile = velocityProfilev1(obj, waypoints, startp,
%        endp, fric)
%             %EQUATION USED TO DETERMINE IDEAL SPEEDS AT EACH WAYPOINT
%             ideal_vels = sqrt(fric*9.81*waypoints(:,3)); %fric is the
%             frictional coefficient of the road %Makes sure car does not
%             go above predetermined top speed for i = 1:numel(ideal_vels)
%                 if ideal_vels(i) > obj.topspeed
%                     ideal_vels(i) = obj.topspeed;
%                 end
%             end %To transform ideal velocities at different poses to
%             realistic %values which can be achieved with maximal
%             acceleration ideal_vels(1) = startp.xdot;
%             ideal_vels(numel(ideal_vels)) = endp.xdot; counter = 1;
%             %total_dist = zeros(2, 1); total_dist =
%             zeros(numel(waypoints(:,1)), 1);
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%VEL_CHART HERE
%             for i = 1:(numel(ideal_vels) - 1)
% %                 if i == numel(ideal_vels) % vel_chart(counter,1) =
% ideal_vels(i); %vel_chart contains all the velocities, including the
% start and the end % vel_chart(counter,2) = i; %                     break
% % end
%                 total_dist(counter) = total_dist(counter) +
%                 sqrt(((waypoints(i,1) - waypoints(i + 1,1))^2) +
%                 ((waypoints(i,2) - waypoints(i + 1,2))^2)); if
%                 ideal_vels(i) ~= ideal_vels(i+1)
%                     vel_chart(counter,1) = ideal_vels(i+1); %SHOULD
%                     FIGURE OUT HOW TO PREALLOCATE vel_chart(counter,2) =
%                     i+1; counter = counter + 1;
%                 end
%             end
% %             vel_chart(counter, 1) = ideal_vels(numel(ideal_vels));
% %Setting the last line as the final velocity % vel_chart(counter, 2) =
% numel(ideal_vels);
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TOTAL_DIST
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%MODS NEEDED HERE
%             %SOMEHOW THIS IS TAKING AGES TO EXECUTE %Records the distance
%             to be travelled at a particular velocity %THIS IS COMPLETELY
%             WRONG NEEDS IMMEDIATE CORRECTION
% %             while i <= numel(waypoints(:,3)) %                 while
% ideal_vels(i + 1) == ideal_vels(i) && i <= numel(waypoints(:,3)) %
% total_dist(n) = total_dist(n) + sqrt(((waypoints(i,1) - waypoints(i -
% 1,1))^2) + ((waypoints(i,2) - waypoints(i-1,2))^2)); % i = i + 1; %
% end %                 n = n + 1; % i = i + 1; %             end
%             i = 1; total_time = zeros(counter - 1, 1);
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             CURRENTLY WORKING HERE while i < counter - 1
%                 %FINAL            INITIAL final_vel =
%                 sqrt((vel_chart(i,1)^2) + 2*obj.maxacc*total_dist(i)); if
%                 final_vel > vel_chart(i+1,1)
%                     temp_dist = ((vel_chart(i+1,1)^2) -
%                     (vel_chart(i,1)^2))/(2 * obj.maxacc); total_time(i) =
%                     ((vel_chart(i+1,1) - vel_chart(i,1))/obj.maxacc) +
%                     ((total_dist(i) - temp_dist)/vel_chart(i+1,1));%AHA
%                     THIS IS THE ERROR HERE
%                 else
%                     %temp_dist = ((final_vel^2) - (vel_chart(i)^2))/(2 *
%                     obj.maxacc); total_time(i) = ((final_vel -
%                     vel_chart(i,1))/obj.maxacc); %+ ((total_dist(i) -
%                     temp_dist)/final_vel); vel_chart(i+1,1) = final_vel;
%                     %Case when final_vel < vel_chart(i+1)
%                 end i = i + 1;
%             end
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             i = 1; vel_profile = zeros(numel(waypoints(:,3)), 2); runtime
%             = 0; sample_time = zeros(counter - 1, 1); while i < counter
%                 n = vel_chart(i,2); sample_time = total_time(i) /
%                 (vel_chart(i+1,2) - n); while n < vel_chart(i+1,2)
%                     if n == 1
%                         n = n + 1; continue
%                     end runtime = runtime + sample_time; vel_profile(n,1)
%                     = vel_profile(n-1,1) + obj.maxacc*sample_time;
%                     vel_profile(n,2) = runtime; n = n + 1;
%                 end i = i + 1;
%             end %%%%%%%%%%%%%%%%%%%%%%%%%%NEEDS TO BE LOOKED AT AGAIN,
%             TOO %%%%%%%%%%%%%%%%%%%%%%%%%%CONVOLUTED
%         end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%         function vel_prof = velocityProfilev3(obj, waypoints, startp,
%         goalp, fric) %CONSTANT ACCELERATION FUNCTION
%             %EQUATION USED TO DETERMINE IDEAL SPEEDS AT EACH WAYPOINT
%             ideal_vels = sqrt(fric*9.81*waypoints(:,3)); %fric is the
%             frictional coefficient of the road %Makes sure car does not
%             go above predetermined top speed for i = 1:numel(ideal_vels)
%                 if ideal_vels(i) > obj.topspeed
%                     ideal_vels(i) = obj.topspeed;
%                 end
%             end %To transform ideal velocities at different poses to
%             realistic %values which can be achieved with maximal
%             acceleration ideal_vels(1) = startp.xdot;
%             ideal_vels(numel(ideal_vels)) = goalp.xdot; total_dist =
%             zeros(numel(waypoints(:,1)), 1); vel_chart =
%             zeros(numel(waypoints(:,1)), 3); counter = 1;
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%VEL_CHART AND
%             TOTAL DIST HERE for i = 1:(numel(ideal_vels) - 1)
%                 if i ~= 1
%                     total_dist(counter - 1) = total_dist(counter - 1) +
%                     sqrt(((waypoints(i,1) - waypoints(i + 1,1))^2) +
%                     ((waypoints(i,2) - waypoints(i + 1,2))^2));
%                 end if ideal_vels(i) ~= ideal_vels(i+1)
%                     vel_chart(counter,1) = ideal_vels(i);
%                     vel_chart(counter,2) = i; counter = counter + 1;
%                 end
%             end vel_chart(counter,1) = ideal_vels(numel(ideal_vels));
%             vel_chart(counter,2) = numel(ideal_vels);
% %             for i = 2:(counter) %                 if vel_chart(i,2) ~=
% (vel_chart(i-1,2) + 1) %                     vel_chart(i,2) =
% int16((vel_chart(i,2) + vel_chart(i-1,2))/2); %                 end % end
%             
%             acc_table = zeros(counter - 1, 1); total_time = zeros(counter
%             - 1, 1); vel_prof = zeros(numel(waypoints(:,3)), 2); for i =
%             1:(counter - 1)%%%TODO - EXAMINE THIS
%                 acc_table(i) = ((vel_chart(i+1,1)^2) -
%                 (vel_chart(i,1)^2))/(2*total_dist(i)); if acc_table(i) >
%                 obj.maxacc
%                     acc_table(i) = obj.maxacc; vel_chart(i+1,1) =
%                     sqrt((vel_chart(i,1)^2) +
%                     2*obj.maxacc*total_dist(i)); %This bit can be
%                     improved for more efficient trajectories
%                 end total_time(i) = (vel_chart(i+1,1) -
%                 vel_chart(i,1))/acc_table(i); sample_time =
%                 total_time(i)/(vel_chart(i+1,2) - vel_chart(i,2)); n =
%                 vel_chart(i,2); while n < vel_chart(i+1,2)
%                     vel_prof(n,1) = vel_chart(i,1) +
%                     acc_table(i)*(n*sample_time); vel_prof(n,2) = n *
%                     sample_time; n = n + 1;
%                 end
%             end
%         end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%         function vel_prof = velocityProfilev2(obj, waypoints, startp,
%         goalp, fric)
%             %EQUATION USED TO DETERMINE IDEAL SPEEDS AT EACH WAYPOINT
%             ideal_vels = sqrt(fric*9.81*waypoints(:,3)); %fric is the
%             frictional coefficient of the road %Makes sure car does not
%             go above predetermined top speed for i = 1:numel(ideal_vels)
%                 if ideal_vels(i) > obj.topspeed
%                     ideal_vels(i) = obj.topspeed;
%                 end
%             end %To transform ideal velocities at different poses to
%             realistic %values which can be achieved with maximal
%             acceleration ideal_vels(1) = startp.xdot;
%             ideal_vels(numel(ideal_vels)) = goalp.xdot; total_dist =
%             zeros(numel(waypoints(:,1)), 1); vel_chart =
%             zeros(numel(waypoints(:,1)), 3); counter = 1;
%             
%             for i = 1:(numel(ideal_vels) - 1)
%                 if i ~= 1
%                     total_dist(counter - 1) = total_dist(counter - 1) +
%                     sqrt(((waypoints(i,1) - waypoints(i + 1,1))^2) +
%                     ((waypoints(i,2) - waypoints(i + 1,2))^2));
%                 end if ideal_vels(i) ~= ideal_vels(i+1)
%                     vel_chart(counter,1) = ideal_vels(i);
%                     vel_chart(counter,2) = i; counter = counter + 1;
%                 end
%             end vel_chart(counter,1) = ideal_vels(numel(ideal_vels));
%             vel_chart(counter,2) = numel(ideal_vels);
%             
%             n = 0; for i = 1:(counter - 2)
%                 n = vel_chart(counter,2); while n < vel_chart(counter +
%                 1, 2)
%                     
%                 end
%             end
%        end

%       TODO - FIX THIS
%         function vel_prof = variable_velprof(obj, waypoints, startp,
%         endp, fric)
%             dist_chart = zeros(numel(waypoints(:,1))); for i =
%             2:numel(waypoints(:,1))
%                 dist_chart(i) = dist_chart(i-1) + sqrt(((waypoints(i,1) -
%                 waypoints(i-1,1))^2) + ((waypoints(i,2) -
%                 waypoints(i-1,2))^2));
%             end syms t eqn = dist_chart == startp.xdot*t +
%             0.5*startp.acc*(t^2) + (1/6)*obj.
%         end
% SNIPPET FROM FIFTH ORDER TRACK DEBUGGING for i = 1:(N-1)
%                 vel_prof(i) = sqrt((fric*9.81)/abs(waypoints(i,3)));
%                 last_dist = waypoints(i,5);
%             end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         function velprof = velocityProfilev4(obj, waypoints, startp,
%         goalp, fric)
%            %EQUATION USED TO DETERMINE IDEAL SPEEDS AT EACH WAYPOINT
%             ideal_vels = sqrt(fric*9.81*waypoints(:,3)); %fric is the
%             frictional coefficient of the road %Makes sure car does not
%             go above predetermined top speed for i = 1:numel(ideal_vels)
%                 if ideal_vels(i) > obj.topspeed
%                     ideal_vels(i) = obj.topspeed;
%                 end
%             end %To transform ideal velocities at different poses to
%             realistic %values which can be achieved with maximal
%             acceleration ideal_vels(1) = startp.xdot;
%             ideal_vels(numel(ideal_vels)) = goalp.xdot; dist_count =
%             zeros((numel(ideal_vels) - 1),1); time_count = dist_count;
%             jerk = 0; for i = 1:(numel(ideal_vels) - 1)
%                 dist_count(i) = sqrt(((waypoints(i,1) -
%                 waypoints(i+1,1))^2) + ((waypoints(i,2) -
%                 waypoints(i+1,2))^2));
%             end
%             
%             for i = 1:(numel(ideal_vels) - 1)
%                 if ideal_vels(i+1) ~= ideal_vels(i)
%                     jerk_sign = ideal_vels(i+1) - ideal_vels(i);
%                     jerk_sign = jerk_sign/abs(jerk_sign); switch
%                     jerk_sign
%                         case 1
%                             jerk = obj.maxjerk;
%                         case -1
%                             jerk = -obj.maxjerk;
%                     end
%                 end syms t eqn = ((1/6)*jerk*(t^3) + 0.5*startp.acc*(t^2)
%                 + startp.xdot*t - dist_count(i) == 0); time = solve(eqn,
%                 t); time_count(i) = time(1);
%             end
%         end
        
        function waypoints = fifthorderTrack(obj, start, goal)
            %Fifth Order Polynomial Equation
            %y(x) = a1x^5 + a2x^4 + a3x^3 + a4x^2 + a5x + ax
            %y'(x) = 5a1x^4 + 4a2x^3 + 3a3x^2 + 2a4x + a5
            %y''(x) = 20a1x^3 + 12a2x^2 + 6a3x +2a4
            N = 200; %Number of points in the trajectory
            xf = goal.x;
            xi = start.x;
            syms a b c d e f
            coeff = [a, b, c, d, e, f]'; %Array of Coefficients to be found
%             max_curve = 1/obj.turnrad;
            waypoints = zeros(N,5);
%             start_y = start.y;
%             start_x = start.x;
            
            B = [start.y, tan(start.psi), 0, goal.y, tan(goal.psi), 0]'; %Array of initial conditions
            A = [xi^5, xi^4, xi^3, xi^2, xi, 1;
                5*xi^4, 4*xi^3, 3*xi^2, 2*xi, 1, 0;
                20*xi^3, 12*xi^2, 6*xi, 2, 0, 0;
                xf^5, xf^4, xf^3, xf^2, xf, 1;
                5*xf^4, 4*xf^3, 3*xf^2, 2*xf, 1, 0;
                20*xf^3, 12*xf^2, 6*xf, 2, 0, 0];
            
            eqn1 = (coeff == (A^(-1))*B);
            
            cof = solve(eqn1, [a, b, c, d, e, f]);
            x_dist = linspace(xi, xf, N);
            for i = 1:N
                waypoints(i,1) = cof.a*x_dist(i)^5 + cof.b*x_dist(i)^4 + cof.c*x_dist(i)^3 + cof.d*x_dist(i)^2 + cof.e*x_dist(i) + cof.f; %Y Coordinates
                waypoints(i,2) = rad2deg(atan(5*cof.a*x_dist(i)^4 + 4*cof.b*x_dist(i)^3 + 3*cof.c*x_dist(i)^2 + 2*cof.d*x_dist(i) + cof.e)); %Slope
                waypoints(i,3) = 20*cof.a*x_dist(i)^3 + 12*cof.b*x_dist(i)^2 + 6*cof.c*x_dist(i) + 2*cof.d; %Curvature
                waypoints(i,4) = x_dist(i); %X Coordinates
            end
            last_dist = 0;
            for i = 1:(N-1) %Cumulative Longitudinal Distance for the velocity planner
%                 waypoints(i,5) = last_dist + sqrt(((waypoints(i,1) - waypoints(i+1,1))^2) + ((waypoints(i,4) - waypoints(i+1,4))^2));
                %waypoints(i,5) = last_dist + (waypoints(i,4) - waypoints(i+1,4));  
                %waypoints(i,5) = waypoints(i,4);
%                 last_dist = waypoints(i, 5);
                waypoints(i,5) = sqrt((waypoints(end,1) - waypoints(1,1))^2 + (waypoints(end,4) - waypoints(1,4))^2);
            end
        end
        
        function [dist, time_arr, vel_prof, acc_prof] = velocityProfilerv5(obj, waypoints, start, goal, fric)
            [minc, maxc] = bounds(waypoints(:,3));
            minc = sqrt((fric*9.81)/abs(minc));
            maxc = sqrt((fric*9.81)/abs(maxc));
            if maxc > minc
                maxv = maxc;
            else
                maxv = minc;
            end
            N = numel(waypoints(:,1));
            jerk = obj.maxjerk; %Maximum jerk
            maxa = obj.maxacc; %Maximum acceleration
            v0 = start.xdot; %Starting velocity
            vf = goal.xdot; %Final velocity
            
            %Scenario Determination
            swcase = 0;
            c1 = (v0/2)*((v0/maxa) + (maxa/jerk));
            if (v0 >= ((maxa^2)/jerk) && waypoints(N-1,5) > c1) || (v0 < ((maxa^2)/jerk) && waypoints(N-1,5) > v0*sqrt(v0/jerk))
                s03 = (maxv + v0)*sqrt((maxv - v0)/jerk);
                s47 = (maxv*0.5)*(((maxv - 2*vf)/maxa)+(maxa/jerk));
                if maxv <= (v0 + ((maxa^2)/jerk)) && waypoints(N-1,5) >= (s03 + s47)
                    swcase = 3; %Case C
                else
                    c1 = (2*maxa*((maxa/jerk)^2) + 5*v0*(maxa/jerk) + 2*(v0^2)/maxa);
                    if waypoints(N-1,5) >= c1
                        swcase = 2; %Case B
                    elseif maxv >= (v0 + ((maxa^2)/jerk))
                        swcase = 1; %Case A
                    else
                        swcase = 4; %Case D
                    end
                end
            else
                swcase = 5; %Case E
            end
            
            %Trajectory Determination
            switch swcase
                case 1 %Case A
                    %Acceleration part
                    t1i = maxa/jerk;
                    t3i = t1i;
                    v1 = v0 + ((maxa^2)/(2*jerk));
                    v2 = maxv - ((maxa^2)/(2*jerk));
                    t2i = (v2 - v1)/maxa;
                    s03 = 0.5*maxv*(((maxv - 2*v0)/maxa) + (maxa/jerk));
                    %Deacceleration part
                    t7i = t1i;
                    t5i = t1i;
                    v6 = vf + ((maxa^2)/(2*jerk));
                    v5 = v2;
                    t6i = (v5 - v6)/maxa;
                    s47 = 0.5*maxv*(((maxv - 2*vf)/maxa) + (maxa/jerk));
                    t4i = (waypoints(N-1,5) - s03 - s47)/maxv;
                    %Outputs
                    total_time = (t1i + t2i + t3i + t4i + t5i + t6i + t7i);
                    t1 = linspace(0, t1i, round(N * t1i/total_time));
                    t2 = linspace(0, t2i, round(N * t2i/total_time));
                    t3 = linspace(0, t3i, round(N * t3i/total_time));
                    t4 = linspace(0, t4i, round(N * t4i/total_time));
                    t5 = linspace(0, t5i, round(N * t5i/total_time));
                    t6 = linspace(0, t6i, round(N * t6i/total_time));
                    t7 = linspace(0, t7i, round(N - N * ((total_time - t7i)/total_time)));
                    
                case 2 %Case B
                    %Acceleration part
                    t1i = maxa/jerk;
                    t3i = t1i;
                    v1 = v0 + ((maxa^2)/(2*jerk));
                    syms V3 V2 T2;
                    eqn1 = [(V2 == v1 + maxa*T2),
                            (v1 + V2 == V3),
                            (waypoints(N-1,5) == (V3*t1i) + (V3*0.5*T2) + (V3*0.5*(((V3 - 2*vf)/maxa) + (maxa/jerk))))];
                    ans1 = solve(eqn1, [V3 V2 T2]);
                    t2i = ans1.T2(ans1.T2 > 0);
                    v3 = ans1.V3(ans1.V3 > 0); %v3 < vmax in this case
                    %Deacceleration part
                    v4 = v3;
                    t7i = t1i;
                    t5i = t1i;
                    v6 = vf + ((maxa^2)/(2*jerk));
                    v5 = v4 - v6;
                    t6i = (v5 - v6)/maxa;
                    %Outputs
                    total_time = (t1i + t2i + t3i + t5i + t6i + t7i);
                    t1 = linspace(0, t1i, round(N * t1i/total_time));
                    t2 = linspace(0, t2i, round(N * t2i/total_time));
                    t3 = linspace(0, t3i, round(N * t3i/total_time));
                    t4 = [];
                    t5 = linspace(0, t5i, round(N * t5i/total_time));
                    t6 = linspace(0, t6i, round(N * t6i/total_time));
                    t7 = linspace(0, t7i, round(N - N * (total_time - t7i)/total_time));
                    
                case 3 %Case C
                    %Acceleration part
                    t1i = sqrt((maxv - v0)/jerk);
                    v1 = v0 + (0.5*jerk*(t1i^2));
                    t3i = t1i;
                    s03 = 2*t1i*v1;                    
                    %Deacceleration part
%                     v4 = maxv;
%                     t7i = maxa/jerk;
%                     t5i = t7i;
%                     v6 = vf + ((maxa^2)/(2*jerk));
%                     v5 = maxv - ((maxa^2)/(2*jerk));
%                     t6i = 0;%(v5 - v6)/maxa;
                    s47 = 0.5*maxv*(((maxv - 2*vf)/maxa) + (maxa/jerk));
                    t4i = (waypoints(N-1,5) - s03 - s47)/maxv;
                    t7i = sqrt((maxv - vf)/jerk);
                    v6 = v0 + (0.5*jerk*(t1i^2));
                    %v3 = maxv;
                    t5i = t7i;
                    %s03 = 2*t1i*v1;
                    %Outputs
                    total_time = (t1i + t3i + t4i + t5i + t7i);
                    %total_time = (t1i + t3i + t4i + t5i + t6i + t7i);
                    t1 = linspace(0, t1i, round(N * t1i/total_time));
                    t2 = [];
                    t3 = linspace(0, t3i, round(N * t3i/total_time));
                    t4 = linspace(0, t4i, round(N * t4i/total_time));
                    t5 = linspace(0, t5i, round(N * t5i/total_time));
                    t6 = [];%linspace(0, t6i, round(N * t6i/total_time));
                    t7 = linspace(0, t7i, round(N - N * (total_time - t7i)/total_time));
                    
                case 4 %Case D
                    %MIGHT NEED TO FIX THIS SINCE THE BELOW EQUATIONS FOR
                    %MAX VELOCITY V3 ARE APPARENTLY COMPUTATIONALLY
                    %EXPENSIVE
                    %Acceleration part
                    syms V3;
                    if v0 >= ((maxa^2)/jerk)
                        eqn = (waypoints(N-1,5) == (V3 + v0)*sqrt((V3 - v0)/jerk) + (0.5*V3)*(((V3 - 2*vf)/maxa) + (maxa/jerk)));
                    else
                        eqn = (waypoints(N-1,5) == (V3 + v0)*sqrt((V3 - v0)/jerk) + (V3 + vf)*sqrt((V3 - vf)/jerk));
                    end
                    ansx = solve(eqn, V3);
                    v3 = ansx;%.V3(ansx.V3 > 0);
                    t1i = sqrt((v3 - v0)/jerk);
                    t3i = t1i;
                    %Deacceleration part
                    t7i = sqrt((v3 - vf)/jerk);
                    t5i = t7i;
                    %Outputs
                    total_time = (t1i + t3i + t5i + t7i);
                    t1 = linspace(0, t1i, round(N * t1i/total_time));
                    t2 = [];
                    t3 = linspace(0, t3i, round(N * t3i/total_time));
                    t4 = [];
                    t5 = linspace(0, t5i, round(N * t5i/total_time));
                    t6 = [];
                    t7 = linspace(0, t7i, round(N - N * (total_time - t7i)/total_time));
                
                case 5 %Case E 
                    %The distance is so small that deacceleration has to be
                    %started immediately
                    %Deacceleration part
                    if v0 >= ((maxa^2)/jerk)
%                         s47 = 0.5*v0*(((v0 - 2*vf)/maxa) + (maxa/jerk));
                        t7i = maxa/jerk;
                        t5i = t7i;
                    else
%                         s47 = (v0 + vf)*sqrt((v0 + vf)/jerk);
                        t7i = sqrt((v0 - vf)/jerk);
                        t5i = t7i;
                    end  
                    %Outputs
                    total_time = (t5i + t7i);
                    t1 = [];
                    t2 = [];
                    t3 = [];
                    t4 = [];
                    t5 = linspace(0, t5i, round(N * t5i/total_time));
                    t6 = [];
                    t7 = linspace(0, t7i, round(N - N * (total_time - t7i)/total_time));
            end
  
            vel_prof = [];
            time_arr = [];
            vel_last = v0;
            a_last = 0;
            dist = [];
            dist_last = 0;
            if ~isempty(t1)
                vel_prof = v0 + 0.5*jerk*(t1.^2);
                vel_last = vel_prof(numel(vel_prof));
                a_last = jerk*t1(numel(t1));
                time_arr = t1;
                acc_prof = jerk*t1;
                
                dist = v0*t1 + (1/6)*jerk*(t1.^3);
                dist_last = dist(numel(dist));
            end
            if ~isempty(t2)
                vel_temp = vel_last + a_last*t2;
                vel_prof = [vel_prof, vel_temp];
                acc_temp = repmat(a_last, 1, numel(t2));
                acc_prof = [acc_prof, acc_temp];
                
                dist_temp = dist_last + vel_last*t2 + (1/2)*a_last*(t2.^2);
                dist_last = dist_temp(numel(dist_temp));
                dist = [dist, dist_temp];
                
                vel_last = vel_temp(numel(vel_temp));
                t_temp = t2 + time_arr(numel(time_arr));
                time_arr = [time_arr, t_temp];
                %acceleration doesnt change in period 2
            end
            if ~isempty(t3)
                vel_temp = vel_last + a_last*t3 - 0.5*jerk*(t3.^2);
                vel_prof = [vel_prof, vel_temp];
                acc_temp = a_last - jerk*t3;
                acc_prof = [acc_prof, acc_temp];
                
                dist_temp = dist_last + vel_last*t3 + (1/2)*a_last*(t3.^2) - (1/6)*jerk*(t3.^3);
                dist_last = dist_temp(numel(dist_temp));
                dist = [dist, dist_temp];
                
                vel_last = vel_temp(numel(vel_temp));
                t_temp = t3 + time_arr(numel(time_arr));
                time_arr = [time_arr, t_temp];
                a_last = 0; %Should be zero at the end of 3rd period, need to test
            end
            if ~isempty(t4)
                %The velocity is constant in this maneuvre, so no need to
                %change vel_last
                vel_temp = repmat(vel_last, 1, numel(t4));
                vel_prof = [vel_prof, vel_temp];
                acc_temp = zeros(numel(t4));
                acc_prof = [acc_prof, acc_temp];
                
                dist_temp = dist_last + vel_last*t4;
                dist_last = dist_temp(numel(dist_temp));
                dist = [dist, dist_temp];
                
                t_temp = t4 + time_arr(numel(time_arr));
                time_arr = [time_arr, t_temp];
                a_last = 0;
            end
            if ~isempty(t5)
                vel_temp = vel_last - 0.5*jerk*(t5.^2);
                vel_prof = [vel_prof, vel_temp];
                acc_temp = a_last - jerk*t5;
                acc_prof = [acc_prof, acc_temp];
                
                dist_temp = dist_last + vel_last*t5 - (1/6)*jerk*(t5.^3);
                dist_last = dist_temp(numel(dist_temp));
                dist = [dist, dist_temp];
                
                vel_last = vel_temp(numel(vel_temp));
                if isempty(time_arr)
                    t_temp = t5;
                else
                    t_temp = t5 + time_arr(numel(time_arr));
                end
                time_arr = [time_arr, t_temp];
                a_last = -jerk*t5(numel(t5));
            end
            if ~isempty(t6)
                vel_temp = vel_last + a_last*t6;
                vel_prof = [vel_prof, vel_temp];
                acc_temp = repmat(a_last, 1, numel(t6));
                acc_prof = [acc_prof,acc_temp];
                
                dist_temp = dist_last + vel_last*t6 - (1/2)*a_last*(t6.^2);
                dist_last = dist_temp(numel(dist_temp));
                dist = [dist, dist_temp];
                
                vel_last = vel_temp(numel(vel_temp));
                t_temp = t6 + time_arr(numel(time_arr));
                time_arr = [time_arr, t_temp];
                %Acceleration doesn't change during period 6
            end
            if ~isempty(t7)
                vel_temp = vel_last + a_last*t7 + 0.5*jerk*(t7.^2);
                vel_prof = [vel_prof, vel_temp];
                acc_temp = a_last + jerk*t7;
                acc_prof = [acc_prof, acc_temp];
                
                dist_temp = dist_last + vel_last*t7 - (1/2)*a_last*(t7.^2) + (1/6)*jerk*(t7.^3);
                dist = [dist, dist_temp];
                
                t_temp = t7 + time_arr(numel(time_arr));
                time_arr = [time_arr, t_temp];
            end
            time_arr = double(time_arr);
            vel_prof = double(vel_prof);
            acc_prof = double(acc_prof);
            dist = double(dist);
        end
        
    end
end
