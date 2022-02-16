function [dist, time_arr, vel_prof, acc_prof, swcase] = velocityProfilerv5_TEST(cum_dist, startx, goalx)
%     [minc, maxc] = bounds(waypoints(:,3));
%     minc = sqrt((fric*9.81)/abs(minc));
%     maxc = sqrt((fric*9.81)/abs(maxc));
%     if maxc > minc
%         maxv = maxc;
%     else
%         maxv = minc;
%     end
    maxv = 10;
    N = 200;
    jerk = 0.6; %Maximum jerk
    maxa = 1.8; %Maximum acceleration
    v0 = startx; %Starting velocity
    vf = goalx; %Final velocity
    
    %Scenario Determination
    swcase = 0;
    c1 = (v0/2)*((v0/maxa) + (maxa/jerk));
    if (v0 >= ((maxa^2)/jerk) && cum_dist > c1) || (v0 < ((maxa^2)/jerk) && cum_dist > v0*sqrt(v0/jerk))
        s03 = (maxv + v0)*sqrt((maxv - v0)/jerk);
        s47 = (maxv*0.5)*(((maxv - 2*vf)/maxa)+(maxa/jerk));
        if maxv <= (v0 + ((maxa^2)/jerk)) && cum_dist >= (s03 + s47)
            swcase = 3; %Case C
        else
            c1 = (2*maxa*((maxa/jerk)^2) + 5*v0*(maxa/jerk) + (2*(v0^2))/maxa);
%             delta_t2 = 0.5*sqrt((((maxa/jerk) - (v0/maxa))^2) + (4*cum_dist/maxa)) - ((maxa^2)/jerk) - 0.5*v0;
            s03_a = maxv*0.5*(((maxv - 2*v0)/maxa) + maxa/jerk);
            if cum_dist >= s03_a + s47 && maxv >= (v0 + ((maxa^2)/jerk))
                swcase = 1; %Case A
            elseif cum_dist >= c1 
                swcase = 2; %Case B
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
            t4i = (cum_dist - s03 - s47)/maxv;
            %Outputs
            if t4i < 0
                t4i = 0;
            end
            total_time = (t1i + t2i + t3i + t4i + t5i + t6i + t7i);
            t1 = linspace(0, t1i, round(N * t1i/total_time));
            t2 = linspace(0, t2i, round(N * t2i/total_time));
            t3 = linspace(0, t3i, round(N * t3i/total_time));
            t4 = linspace(0, t4i, round(N * t4i/total_time));
            t5 = linspace(0, t5i, round(N * t5i/total_time));
            t6 = linspace(0, t6i, round(N * t6i/total_time));
            t7 = linspace(0, t7i, round(N * t7i/total_time));
            
        case 2 %Case B
            %Acceleration part
            t1i = maxa/jerk;
            t3i = t1i;
            v1 = v0 + ((maxa^2)/(2*jerk));
            syms V3 V2 T2;
            eqn1 = [(V2 == v1 + maxa*T2),
                    (v1 + V2 == V3),
                    (cum_dist == (V3*t1i) + (V3*0.5*T2) + (V3*0.5*(((V3 - 2*vf)/maxa) + (maxa/jerk))))];
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
            v5 = maxv - ((maxa^2)/(2*jerk));
    %                     t6i = 0;%(v5 - v6)/maxa;
            s47 = 0.5*maxv*(((maxv - 2*vf)/maxa) + (maxa/jerk));
            t4i = (cum_dist - s03 - s47)/maxv;
            t7i = maxa/jerk;
            v6 = vf + (0.5*jerk*(t7i^2));
            t5i = t7i;
            t6i = (v5 - v6)/maxa;
            %Outputs
            total_time = (t1i + t3i + t4i + t5i + t6i + t7i);
            t1 = linspace(0, t1i, round(N * t1i/total_time));
            t2 = [];
            t3 = linspace(0, t3i, round(N * t3i/total_time));
            t4 = linspace(0, t4i, round(N * t4i/total_time));
            t5 = linspace(0, t5i, round(N * t5i/total_time));
            t6 = linspace(0, t6i, round(N * t6i/total_time));
            t7 = linspace(0, t7i, round(N - N * (total_time - t7i)/total_time));
            
        case 4 %Case D
            %Acceleration part
            syms V3;
            if v0 >= ((maxa^2)/jerk)
                eqn = (cum_dist == (V3 + v0)*sqrt((V3 - v0)/jerk) + (0.5*V3)*(((V3 - 2*vf)/maxa) + (maxa/jerk)));
            else
                eqn = (cum_dist == (V3 + v0)*sqrt((V3 - v0)/jerk) + (V3 + vf)*sqrt((V3 - vf)/jerk));
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
                t7i = maxa/jerk;
                t5i = t7i;
            else
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
            
            t7(1) = [];
    end
    
    vel_prof = [];
    time_arr = [];
    vel_last = v0;
    a_last = 0;
    dist = [];
    dist_last = 0;
    acc_prof = [];
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
        acc_temp = zeros(1, numel(t4));
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
        if ~isempty(acc_prof)
            acc_prof = [acc_prof, acc_temp];
        else
            acc_prof = acc_temp;
        end
        
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