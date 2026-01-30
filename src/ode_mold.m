function dzdt = ode_mold(~, z, r, M, A)
% ODE system for [x; p] under Pontryagin optimality condition.

x = z(1);
p = z(2);

% Optimal control from dH/du = 0
u = 0.5 * p * x;

dzdt = zeros(2,1);
dzdt(1) = r*(M - x) - u*x;          % state equation
dzdt(2) = -2*A*x + p*(r + u);       % adjoint equation
end
