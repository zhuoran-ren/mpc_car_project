clc; clear; close all;
%% 

syms x y theta V real       % State Variables
syms delta u_T real         % Control Variables
syms Vs u_Ts real           % Steady-state Constants

% State Vector
X = [x; y; theta; V];
u = [delta; u_T];

%% 

% Sampling Time
Ts = 1/10;

% Car Class Initialization
car = Car(Ts);

% Dynamics Function
f = car.f(X, u);

% Jacobians Computation
Asym = jacobian(f, X);     % Partial Derivative of f w.r.t. X
Bsym = jacobian(f, u);     % Partial Derivative of f w.r.t. u

% Steady-state values
xs = [0; 0; 0; Vs];
us = [0; u_Ts];

% Substitute steady-state values into A, B, f
A = subs(Asym, [X; u], [xs; us]);
B = subs(Bsym, [X; u], [xs; us]);
fs = subs(f, [X; u], [xs; us]);

% Replacing max([1, abs(Vs)]) with Vs in A_ss, B_ss, and fs
A = subs(A, max([1, abs(Vs)]), Vs);
B = subs(B, max([1, abs(Vs)]), Vs);
fs = subs(fs, max([1, abs(Vs)]), Vs);

%% 

% Further Simplification (increasing steps for deeper simplification)
A = simplify(A, 'Steps', 500);
B = simplify(B, 'Steps', 500);
fs = simplify(fs, 'Steps', 500);

% Display f(xs,us), A, B
disp('f(xs,us):');
disp(fs);
disp('A:');
disp(A);
disp('B:');
disp(B);

% Compute and simplify f_next
f_next = fs + A * (X - xs) + B * (u - us);
f_next = simplify(f_next, 'Steps', 500);

% Numerical approximation for further readability (Approximation to 4 significant digits)
A_cleaned = vpa(A, 4);
B_cleaned = vpa(B, 4);
fs_cleaned = vpa(fs, 4);
f_next_cleaned = vpa(f_next, 4);

% Display f(xs,us), A, B, f_next approximated to 4 significant digits
disp('f(xs,us) (approximation):');
disp(fs_cleaned);

disp('A (approximation):');
disp(A_cleaned);

disp('B (approximation):');
disp(B_cleaned);

disp('f_next (approximation):');
disp(f_next_cleaned);

%% 

% Numerical Evaluation
Vs_value = 120 / 3.6;
u_Ts_value = 0.2018;

% Substituting numerical values into A, B, fs
A_numeric = subs(A_cleaned, [Vs, u_Ts], [Vs_value, u_Ts_value]);
B_numeric = subs(B_cleaned, [Vs, u_Ts], [Vs_value, u_Ts_value]);
fs_numeric = subs(fs_cleaned, [Vs, u_Ts], [Vs_value, u_Ts_value]);

% Rounding to 4 decimal places using vpa
A_numeric_rounded = vpa(A_numeric, 6);
B_numeric_rounded = vpa(B_numeric, 6);
fs_numeric_rounded = vpa(fs_numeric, 6);

% Display results
disp('f(xs,us) (numeric, after substituting Vs and u_Ts):');
disp(fs_numeric_rounded);

disp('A (numeric, after substituting Vs and u_Ts):');
disp(A_numeric_rounded);

disp('B (numeric, after substituting Vs and u_Ts):');
disp(B_numeric_rounded);
