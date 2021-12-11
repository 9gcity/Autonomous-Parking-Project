%Offline function to calculate the entire tables of linearized state 
%functions and control coefficient matrices
function [A, B] = linearization_V2(u1, x3, x4, obj)
    %Empty Variables(in this case)
    A = zeros(5, 5, numel(u1), numel(x3), numel(x4));
    B = zeros(5, 2, numel(u1), numel(x3), numel(x4));
    %const = zeros(4, 1, numel(u1), numel(x3), numel(x4));
    P_mat = zeros(5, 5, numel(u1), numel(x3), numel(x4));
    K = zeros(2, 5, numel(u1), numel(x3), numel(x4));
    %Kff = zeros(2, 1, numel(u1), numel(x3), numel(x4));
    
    %Constants
    C = 1/obj.whlbase;
    Q = diag([1,1,1,1,1]);
    R = diag([1,1]);
    
    %u1, x3, and x4 would consist of the midpoints of all the value regions
    for i = 1:numel(u1)
        for j = 1:numel(x3)
            for k = 1:numel(x4)
                %Values of the state derivatives calculated at the
                %specified operating points
                x1dot = u1(i) * cos(x3(j));
                x2dot = u1(i) * sin(x3(j));
                x3dot = C * u1(i) * tan(x4(k));
                
                %Linearized state space matrices - 2 inputs, 4 outputs
                A(:,:,i,j,k) = [0, 0, (-u1(i)*sin(x3(j))), 0, x1dot + u1(i)*(x3(j)*sin(x3(j)) - cos(x3(j)));
                                0, 0, (u1(i)*cos(x3(j))), 0, x2dot - u1(i)*(sin(x3(j)) + x3(j)*cos(x3(j)));
                                0, 0, 0, u1(i)*C*(sec(x4(k))^2), x3dot - u1(i)*C*(tan(x4(k)) + x4(k)*((sec(x4(k)))^2));
                                0, 0, 0, 0, 0;
                                0, 0, 0, 0, 0];
                A_temp = A(:,:,i,j,k);
                            
                B(:,:,i,j,k) = [cos(x3(j)), 0;
                                sin(x3(j)), 0;
                                C*tan(x4(k)), 0;
                                0, 1;
                                0, 0];
                B_temp = B(:,:,i,j,k);
                
%                 syms P
%                 eqn = (A_temp.')*P + P*A_temp - P*B_temp*inv(R)*(B_temp.')*P + Q == 0;
%                 %ERROR IN SOLVING THIS HERE
%                 P_mat(:,:,i,j,k) = solve(eqn);
                
                %[P_mat(:,:,i,j,k), K(:,:,i,j,k), ~] = icare(A_temp, B_temp, Q, R, [], [], []);
%                 K(:,:,i,j,k) = inv(R)*(B_temp.')*P_mat(:,:,i,j,k);
            end
        end
    end
end
