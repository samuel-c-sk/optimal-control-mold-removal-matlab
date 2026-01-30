%% Optimal Control of Mold Growth (Pontryagin + BVP4C)
% Model:
%   x'(t) = r (M - x) - u(t) x,   x(0) = x0
% Cost functional:
%   J(u) = ∫_0^T (A x(t)^2 + u(t)^2) dt
% Pontryagin condition gives:
%   u*(t) = 0.5 * p(t) * x(t)
% where p(t) is the adjoint variable with boundary condition p(T) = 0.
%
% This script solves the resulting boundary value problem using MATLAB bvp4c
% for multiple parameter sets and compares u(t) and x(t).

clear; clc;

% Parameter sets: [r  M   A   x0  T]
paramSets = [0.3 10  1   1   5;   % set 1
             0.3 10  10  1   5;   % set 2
             0.1 10  10  1   5;   % set 3
             0.6 5   10  5   5];  % set 4

nT = 200;

figure(1); clf; hold on;
figure(2); clf; hold on;

for k = 1:size(paramSets,1)
    r  = paramSets(k,1);
    M  = paramSets(k,2);
    A  = paramSets(k,3);
    x0 = paramSets(k,4);
    T  = paramSets(k,5);

    t = linspace(0,T,nT);

    % Initial guess for BVP: x(t) ~ x0, p(t) ~ 0
    guess = bvpinit(t, [x0 0]);

    % Solve BVP
    sol = bvp4c(@(t,z) ode_mold(t,z,r,M,A), @(ya,yb) bc_mold(ya,yb,x0), guess);

    tPlot = linspace(0,T,nT);
    x = deval(sol, tPlot, 1);
    p = deval(sol, tPlot, 2);
    u = 0.5 * p .* x;

    J = trapz(tPlot, A*x.^2 + u.^2);
    fprintf('Set %d: r=%.2f, M=%.1f, A=%.1f, x0=%.1f, T=%.1f  -->  J = %.4f\n',...
        k,r,M,A,x0,T,J);

    figure(1); plot(tPlot, u, 'LineWidth', 1.5);
    figure(2); plot(tPlot, x, 'LineWidth', 1.5);
end

figure(1);
xlabel('t'); ylabel('u(t)');
title('Optimal control u(t) for different parameter sets');
legend('set 1','set 2','set 3','set 4','Location','best');

figure(2);
xlabel('t'); ylabel('x(t)');
title('Mold amount x(t) for different parameter sets');
legend('set 1','set 2','set 3','set 4','Location','best');

% Export figures for the GitHub repo
if ~exist('../figures','dir'); mkdir('../figures'); end
exportgraphics(figure(1), '../figures/optimal_control_u.png', 'Resolution', 200);
exportgraphics(figure(2), '../figures/state_trajectory_x.png', 'Resolution', 200);
