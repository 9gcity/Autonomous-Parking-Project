%Offline function to calculate the entire tables of linearized state 
%functions and control coefficient matrices
function [A, B, K] = linearization_V3(u1, x3, x4, obj)
    %Empty Variables(in this case)
    A = zeros(4, 4, numel(u1), numel(x3), numel(x4));
    B = zeros(4, 2, numel(u1), numel(x3), numel(x4));
    const = zeros(8, 1, numel(u1), numel(x3), numel(x4));
%     P_mat = zeros(4, 4, numel(u1), numel(x3), numel(x4));
%     K = zeros(2, 4, numel(u1), numel(x3), numel(x4));
%     Kff = zeros(2, 1, numel(u1), numel(x3), numel(x4));
%     alpha = zeros(4,1,numel(u1), numel(x3), numel(x4));
%     beta = zeros(2,1,numel(u1), numel(x3), numel(x4));
   
    %Constants
    C = 1/obj.whlbase;
%     Q = [1, 0;
%         0, 1];
%     R = [1, 0;
%         0, 1];
    
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
                A(:,:,i,j,k) = [0, 0, (-u1(i)*sin(x3(j))), 0;
                                0, 0, (u1(i)*cos(x3(j))), 0;
                                0, 0, 0, u1(i)*C*(sec(x4(k))^2);
                                0, 0, 0, 0];
                A_temp = A(:,:,i,j,k);
                            
                B(:,:,i,j,k) = [cos(x3(j)), 0;
                                sin(x3(j)), 0;
                                C*tan(x4(k)), 0;
                                0, 1];
                B_temp = B(:,:,i,j,k);
                
                const(:,:,i,j,k) = [x1dot + u1(i)*(x3(j)*sin(x3(j)) - cos(x3(j)));
                                    x2dot - u1(i)*(sin(x3(j)) + x3(j)*cos(x3(j)));
                                    x3dot - u1(i)*C*(tan(x4(k)) + x4(k)*((sec(x4(k)))^2));
                                    0;
                                    0;
                                    0;
                                    0;
                                    0];
                
                C_mat = diag([1,1,1,1]);
                D = zeros(4,2);
                
%                 syms a1 a2 a3 a4 b1 b2;
%                 eqn = [A_temp, B_temp; C_mat, D]*[a1;a2;a3;a4;b1;b2] == const_temp;
%                 x = solve(eqn, [a1, a2, a3, a4, b1, b2]);
%                 alpha(:,:,i,j,k) = [x.a1; x.a2; x.a3; x.a4];
%                 beta(:,:,i,j,k) = [x.b1; x.b2];
                sys = ss(A_temp, B_temp, C_mat, D);
                Q = diag([1,1,1,1]);
                R = diag([1,1]);
                [K(:,:,i,j,k), ~, ~] = lqr(sys, Q, R);
            end
        end
    end
end
