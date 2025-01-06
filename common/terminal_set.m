Ts = 1/10;
car = Car(Ts);
Vs = 120/3.6;
[xs, us] = car.steady_state(Vs);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);

% % Test H_lon first
% H_lon = 5;
% mpc_lon = MpcControl_lon(sys_lon, Ts,H_lon);
% u0_lon = mpc_lon.get_u([0, 80/3.6]', 120/3.6);

H_lon = 5;
H_lat = 20;
mpc_lat = MpcControl_lat(sys_lat,Ts, H_lon);
mpc= car.merge_lin_controllers(mpc_lon, mpc_lat);

x_ref = [3; 0];

F = [1, 0; -1, 0; 0, 1; 0, -1];
f = [3.5; 0.5; 0.0873; 0.0873];
M = [1; -1];
m = [0.5236; 0.5236];
A = mpc_lat.A;
B = mpc_lat.B;

Q = 50 * eye(2);
R = 1 * eye(1);
[K, Qf, ~] = dlqr(A, B, Q, R);
K = -K;

% Compute the terminal set offline

Xf = polytope([F; M*K], [(f  - F * x_ref); m]);
Acl = A + B*K;
while 1
    prevXf = Xf;
    [T, t] = double(Xf);
    Xf_new = polytope(T*Acl, t-T*(A-eye(2))*x_ref);
    Xf = intersect(Xf, Xf_new);
    if isequal(prevXf, Xf)
        break
    end
end
[Ff, ff] = double(Xf);
plot(Xf);

% Visualizing the sets
% figure
% hold on; grid on;
% plot(polytope(Ff_init, ff_init), 'g'); plot(Xf, 'r');

%% Save the terminal set
save('terminal_set.mat', 'Ff', 'ff');