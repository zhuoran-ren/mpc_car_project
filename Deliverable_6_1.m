addpath("common\")
addpath("Deliverable_6_1\")

Ts = 1/10;
car = Car(Ts);
H = 5;
mpc = NmpcControl(car, H);

%% Check in the open loop
% x0 = [0 0 0 80/3.6];
% ref = [3 100/3.6];
% u = mpc.get_u(x0, ref);

%% Check in the close loop
% params = {};
% params.Tf = 15;
% params.myCar.model = car;
% params.myCar.x0 = [0 0 0 80/3.6];
% params.myCar.u = @mpc.get_u;
% ref1 = [0 80/3.6]';
% ref2 = [0 120/3.6]';
% params.myCar.ref = car.ref_step(ref1, ref2, 2);
% 
% result = simulate(params);
% visualization(car, result);