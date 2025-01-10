clc; clear; close all;

Ts = 1/10;
car = Car(Ts);
H = 10;

mpc = NmpcControl(car, H);
%% Open Loop
% x0 = [0 0 0 80/3.6]';
% ref = [3 100/3.6]';
% u = mpc.get_u(x0, ref);
%% Closed Loop
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 80/3.6]';
params.myCar.u = @mpc.get_u;
ref1 = [0 80/3.6]';
ref2 = [3 100/3.6]';
params.myCar.ref = car.ref_step(ref1, ref2, 2);
result = simulate(params);
visualization(car, result);
