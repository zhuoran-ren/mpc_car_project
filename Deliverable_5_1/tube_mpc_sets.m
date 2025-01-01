addpath("..\common\")

% Caculate all components of your controller
Ts = 1/10;
car = Car(Ts);
Vs = 120/3.6;
[xs, us] = car.steady_state(Vs);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
[~, A, B, ~, ~] = Car.c2d_with_offset(sys_lon, Ts);
us_lon = us(2);
% We only care about the sys_lon
%% Caculate the minimum robust invariant set
% Nothing to do with the controller in this file
% And nothing to do with the input in this file
[nx, nu] = size(B);
poles = [0.85, 0.90];
K = -place(A, -B, poles);
% K = [0.3, 0.64];
fprintf("%g", eigs(A - B* K));
Q = 10 * eye(nx);
R = 1 * eye(nu);
% [K, Qf, ~] = dlqr(A, -B, Q, R);
% K = -K;

A_bar = A - B * K;
T = [1; -1];
t = [0.5; 0.5];
P_w = B * Polyhedron(T, t);
% epolision = P_w;
% i = 1;
% while 1
%     epolision = epolision + A_bar ^ i * P_w;
%     epolision = minHRep(epolision);
%     err = norm(A_bar^i);
%     disp(err);
%     if err < 1e-2
%         break
%     end
%     i = i+1;
% end
% plot(epolision);
E = P_w;
i = 1;
while true
    E = E + A_bar^i*P_w; % if E and W are Polyhedron, the operator + acts as Minkowski sum and not normal addition in MPT
    E = minHRep(E);
    if norm(A_bar^i) < 1e-2   %check if diff is sufficiently small to terminate
        fprintf("minimum robust invariant set passed vibe check after %i iterations", i)
            break;
    end
    i = i+1;
    fprintf('iteration %i and norm %i \n',i, norm(A_bar^i));
end
T_E = E.A;
t_E = E.b;
plot(E);
% P_plus_Q is the Minimum Robust Invariant Set
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

%% Caculate the terminal components
P = dlyap(A_bar',Q + K'*R*K);
% Xf = Polyhedron(F_tight, f_tight);
Xf = Polyhedron([F_tight; M_tight*K],[f_tight; m_tight]);
plot(Xf);
while 1
    disp('Hi');
    prevXf = Xf;
    T_Xf = Xf.A;
    t_Xf = Xf.b;
    new_Xf = Polyhedron(T_Xf*A_bar,t_Xf);
    Xf = intersect(Xf, new_Xf);
    if abs(Xf.volume - prevXf.volume) < 1e-10
        break
    end
end
plot(Xf);

%% Save the final value
save('tube_mpc_data.mat', 'F_tight', 'M_tight', 'f_tight', 'm_tight', 'T_Xf', 't_Xf', 'P', 'T_E', 't_E', 'K', 'Q', 'R');

