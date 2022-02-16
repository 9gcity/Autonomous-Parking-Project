% function y = carstatefunc(x,u)
%     y = zeros(5,1);
%     y(1) = x(5)*cos(x(3));
%     y(2) = x(5)*sin(x(3));
%     y(3) = (x(5)*tan(x(4)))/2.551;
%     y(4) = u(1);
%     y(5) = x(6);
%     y(6) = u(2);
% end

function y = carstatefunc(x,u)
    y = zeros(5,1);
    y(1) = x(5)*cos(x(3));
    y(2) = x(5)*sin(x(3));
    y(3) = (x(5)*tan(x(4)))/2.551;
    y(4) = u(1);
    y(5) = x(6);
    y(6) = u(2);
end