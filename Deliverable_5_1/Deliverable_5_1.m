addpath("common\")
addpath("Deliverable_5_1\")

Ts = 1/10;
car = Car(Ts);
Vs = 120/3.6;
[xs, us] = car.steady_state(Vs);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);

H_lon = 25;
H_lat = 5;
mpc_lon = MpcControl_lon(sys_lon,Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat,Ts, H_lat);
mpc= car.merge_lin_controllers(mpc_lon, mpc_lat);

%% Test in open loop
% x0 = [0 80/3.6]';
% x0other = [15 100/3.6]';
% ref = 100/3.6;
% [u_lon, X_lon, U_lon] = mpc_lon.get_u(x0,ref, x0other);

%% Test in close loop
params= {};
otherRef = 120/3.6;
x0=[0 0 0 115/3.6]';%(x,y,theta,V)
params.Tf=25;
params.myCar.model=car;
params.myCar.x0=x0;
params.myCar.u= @mpc.get_u;
params.myCar.ref= [0 120/3.6]';%delayreferencestepby5s

% Something relate to lead car
params.otherCar.model = car;
params.otherCar.x0= [8 0 0 otherRef]';
params.otherCar.u = car.u_fwd_ref();
params.otherCar.ref = car.ref_robust();


result=simulate(params);
visualization(car,result)
