addpath("common\")
addpath("Delivearble_4_1\")

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
mpc_lon = MpcControl_lon(sys_lon,Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat,Ts, H_lon);
mpc= car.merge_lin_controllers(mpc_lon, mpc_lat);

% Create the estimator
estimator = LonEstimator(sys_lon, Ts);

x0=[0 0 0 80/3.6]';%(x,y,theta,V)
ref1=[0 80/3.6]'; %(yref, Vref)
ref2=[3 50/3.6]'; %(yref, Vref)
params= {};
params.Tf=15;
params.myCar.model=car;

% add the estimate function and estimate value
params.myCar.est_fcn = @estimator.estimate;
params.myCar.est_dist0 = 0;

params.myCar.x0=x0;
params.myCar.u= @mpc.get_u;
params.myCar.ref= car.ref_step(ref1,ref2,2);%delayreferencestepby5s
result=simulate(params);
visualization(car,result)
