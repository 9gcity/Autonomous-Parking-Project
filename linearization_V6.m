%Offline function to calculate the entire tables of linearized state 
%functions and control coefficient matrices
function [A, B, Kd1, Kd2, Kx, const] = linearization_V6(x3, x4, x5, obj, Q, R)
    %Empty Variables(in this case)
    A = zeros(5, 5, numel(x5), numel(x3), numel(x4));
    B = zeros(5, 2, numel(x5), numel(x3), numel(x4));
    const = zeros(5, 1, numel(x5), numel(x3), numel(x4));
    
    Kd1 = zeros(2, 5, numel(x5), numel(x3), numel(x4));
    Kx = zeros(2, 5, numel(x5), numel(x3), numel(x4));
    Kd2 = zeros(2, 5, numel(x5), numel(x3), numel(x4));
    
    %Constants
    C = 1/obj.whlbase;
    C_mat = diag([1,1,1,1,1]);
    D = zeros(5,2);
    
    %u1, x3, and x4 would consist of the midpoints of all the value regions
    for i = 1:numel(x5)
        for j = 1:numel(x3)
            for k = 1:numel(x4)
                %Values of the state derivatives calculated at the
                %specified operating points
                x1dot = x5(i) * cos(x3(j));
                x2dot = x5(i) * sin(x3(j));
                x3dot = C * x5(i) * tan(x4(k));
                
                %Linearized state space matrices - 2 inputs, 4 outputs
                A(:,:,i,j,k) = [0, 0, (-x5(i)*sin(x3(j))), 0, cos(x3(j));
                                0, 0, (x5(i)*cos(x3(j))), 0, sin(x3(j));
                                0, 0, 0, x5(i)*C*(sec(x4(k))^2), C*tan(x4(k));
                                0, 0, 0, 0, 0;
                                0, 0, 0, 0, 0];
                                
                A_temp = A(:,:,i,j,k);
                            
                B(:,:,i,j,k) = [0, 0;
                                0, 0;
                                0, 0;
                                1, 0;
                                0, 1];
                            
                B_temp = B(:,:,i,j,k);
                
                const(:,:,i,j,k) = [x1dot + x5(i)*(x3(j)*sin(x3(j)) - cos(x3(j)));
                                    x2dot - x5(i)*(sin(x3(j)) + x3(j)*cos(x3(j)));
                                    x3dot - x5(i)*C*(tan(x4(k)) + x4(k)*((sec(x4(k)))^2));
                                    0;
                                    0];
                P = sym('p',[5,5]);
                eqn = P*A_temp - P*B_temp*inv(R)*(B_temp')*P + (C_mat')*Q*C_mat + (A_temp')*P == 0;
                
                P_f = solve(eqn, P);
                Kx(:,:,i,j,k) = -inv(R)*(B_temp')*P_f;
                
                h0 = inv(P*B_temp*inv(R)*(B_temp') - A_temp')*P_f*B_temp;
                Kd1(:,:,i,j,k) = -inv(R)*(B_temp')*h0;
                
                h1 = inv(P*B_temp*inv(R)*(B_temp') - A_temp')*h0;
                Kd2(:,:,i,j,k) = -inv(R)*(B_temp')*h1;
%                 r_temp = zeros(8,8);
%                 r_temp(6,1) = -1;
%                 r_temp(7,2) = -1;
%                 r_temp(8,3) = -1;
%                 r(:,:,i,j,k) = r_temp;
%                 syms a1 a2 a3 a4 b1 b2;
%                 eqn = [A_temp, B_temp; C_mat, D]*[a1;a2;a3;a4;b1;b2] == const_temp;
%                 x = solve(eqn, [a1, a2, a3, a4, b1, b2]);
%                 alpha(:,:,i,j,k) = [x.a1; x.a2; x.a3; x.a4];
%                 beta(:,:,i,j,k) = [x.b1; x.b2];
%                 sys = ss(A_temp, B_temp, C_mat, D);
                %Q = diag([1,1,1,1,1]);
                %R = diag([1,1]);
%                 [Kd(:,:,i,j,k), ~, e(:,i,j,k)] = lqr(sys, Q, R);
                
            end
        end
    end
end
