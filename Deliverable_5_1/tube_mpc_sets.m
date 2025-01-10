clc; clear; close all;
addpath("..\common\")

Ts = 1/10;
car = Car(Ts);
Vs = 120/3.6;
[xs, us] = car.steady_state(Vs);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
[~, A, B, ~, ~] = Car.c2d_with_offset(sys_lon, Ts);
us_lon = us(2);
%% Caculate the minimum robust invariant set
[nx, nu] = size(B);
Q = 50 * eye(nx);
R = 1 * eye(nu);
[K, P, ~] = dlqr(A, -B, Q, R);
K = -K;

A_bar = A - B * K;
T = [1; -1];
t = [0.5; 0.5];
P_w = B * Polyhedron(T, t);
E = P_w;
i = 1;
while true
    E = E + A_bar^i*P_w;
    E = minHRep(E);
    if norm(A_bar^i) < 1e-2
        fprintf("Get our invarience set after %i iterations\n", i)
        break;
    end
    fprintf("iteration %i to get our invariant set\n",i);
    i = i+1;
end
T_E = E.A;
t_E = E.b;
%% Caculate the tightenen constraints
F_x = [-1 0];
f_x = 4;
P_x = Polyhedron(F_x, f_x);
tightened_P_x = P_x - E;

F_u = [1; -1];
f_u = [1 - us_lon; 1 + us_lon];
P_u = Polyhedron(F_u, f_u);
tightened_P_u = P_u - K * E;

F_tight = tightened_P_x.A;
M_tight = tightened_P_u.A;
f_tight = tightened_P_x.b;
m_tight = tightened_P_u.b;

%% Caculate the terminal set
Xf = Polyhedron([F_tight; M_tight*K],[f_tight; m_tight]);
i = 1;
while 1
    prevXf = Xf;
    T_Xf = Xf.A;
    t_Xf = Xf.b;
    new_Xf = Polyhedron(T_Xf*A_bar,t_Xf);
    Xf = intersect(Xf, new_Xf);
    if abs(Xf.volume - prevXf.volume) < 1e-10
        fprintf("Get our terminal set after %i iterations\n", i)
        break
    end
    fprintf("iteration %i to get our Terminal set\n",i);
    i = i + 1;
end

%% Plot the set if needed
figure;
plot(E);
title('Invarient set');
xlabel('x1: deltax');
ylabel('x2: deltav');

figure;
plot(Xf);
title('Terminal set');
xlabel('x1: deltax');
ylabel('x2:deltav');
%% Save the final value
save('tube_mpc_data.mat', 'F_tight', 'M_tight', 'f_tight', 'm_tight', 'T_Xf', 't_Xf', 'P', 'T_E', 't_E', 'K', 'Q', 'R');

