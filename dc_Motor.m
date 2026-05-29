% DC Motor Position Control using PID
clc; clear; close all;

% Parameters
J = 0.02;   % moment of inertia (kg.m^2)
b = 0.05;   % damping coefficient (N.m.s)
K = 0.02;   % motor torque constant & back EMF constant
R = 2;      % armature resistance (Ohm)
L = 1;      % armature inductance (H)

% Step 1: Motor Transfer Function
num = [K];
den = [J*L  (J*R + L*b)  (R*b + K^2)];
sys = tf(num, den);
disp('DC Motor Transfer Function:');
sys

% Step 2: PID Controller with fixed gains
Kp = 32.4;
Ki = 25.87;
Kd = 4.68;

C_PID = pid(Kp, Ki, Kd);

fprintf('--- Fixed PID Gains ---\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);

% Step 3: Closed-loop system with PID
T_PID = feedback(C_PID*sys,1);

% Step 4: Step Response and Performance
figure;
step(T_PID)
grid on
title(sprintf('Step Response with PID Controller (Kp=%.3f, Ki=%.3f, Kd=%.4f)', Kp, Ki, Kd))
ylabel('Motor Position (deg)')
xlabel('Time (s)')

% Get performance info
info = stepinfo(T_PID);
disp('Step Response Characteristics:')
disp(info)

% Final Value
final_value = dcgain(T_PID);
fprintf('Final Value = %.2f deg\n', final_value*90); % scaled to 90 deg input

% Step 5: Disturbance Test (external torque at t=2s)
t = 0:0.01:5;               % time vector
u = 90*ones(size(t));       % reference input = 90 deg
disturbance = zeros(size(t));
disturbance(t>=2) = -10;    % disturbance at t=2s (−10 input)

u_total = u + disturbance;  % total input

[y,t_out] = lsim(T_PID,u_total,t);

figure;
plot(t_out,y,'b','LineWidth',1.5); hold on;
plot(t,u,'r--','LineWidth',1.2);
legend('Motor Output','Reference (90°)')
title('Disturbance Test at t=2s')
xlabel('Time (s)')
ylabel('Motor Position (deg)')
grid on

% Step 6: Controller Comparison (P, PI, PID) using same system
C_P  = pidtune(sys,'P');
C_PI = pidtune(sys,'PI');

T_P  = feedback(C_P*sys,1);
T_PI = feedback(C_PI*sys,1);

figure;
step(T_P,'r',T_PI,'g',T_PID,'b')
legend('P','PI','PID')
grid on
title('Controller Comparison (P vs PI vs PID)')
xlabel('Time (s)')
ylabel('Motor Position (deg)')