function res = bc_mold(ya, yb, x0)
% Boundary conditions: x(0) = x0, p(T) = 0

res = [ya(1) - x0;
       yb(2)];
end
