function xnext = carstatefuncDiscrete(xi, ui, Ts)
    xi = xi(:);
    ui = ui(:);
    statefunc = @(xi,ui) carstatefunc(xi,ui);
    fi = statefunc(xi,ui);
    
    xnext = xi + Ts*fi;
    
    % Solve for xnext satisfying the Trapezoidal rule.
    FUN = @(xnext) TrapezoidalRule(xi,xnext,ui,Ts,fi,statefunc);
    Options = optimoptions('fsolve','Display','none');
    xnext = fsolve(FUN,xnext,Options);

% Trapezoidal rule function
function f = TrapezoidalRule(xk,xk1,uk,Ts,fk,ffun)
% Time derivatives at point xk1.
fk1 = ffun(xk1,uk);
% The following must be zero to satisfy the fsolve constraints for the 
% Trapezoidal Rule
f = xk1 - (xk + (Ts/2)*(fk1 + fk));
