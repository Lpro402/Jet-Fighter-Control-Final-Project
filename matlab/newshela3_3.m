%% ============================================================
% Question 3(c)
% Observer-based state feedback using roll-rate measurement p
%% ============================================================

clear
close all
clc

%% Aircraft model
% State vector:
% x = [beta  p  r  phi  delta_a]'

A = [ ...
    -0.575     0       -1       0.0536   -0.078;
    -300      -3.03     2       0        64.4;
      68       0.045   -2.4     0         5;
       0       1        0       0         0;
       0       0        0       0        -5];

B = [0; 0; 0; 0; 5];

C_phi = [0 0 0 1 0];
C_p   = [0 1 0 0 0];

n = size(A,1);

%% ============================================================
% 1. Controller design from Question 3(a)
%% ============================================================

% Selected controller poles
controller_poles_desired = [ ...
    -1.8 + 2.4i;
    -1.8 - 2.4i;
    -2.1 + 2.14242853i;
    -2.1 - 2.14242853i;
    -2.05];

% Controllability check
Co = ctrb(A,B);
rank_Co = rank(Co);

fprintf('Controllability rank = %d\n',rank_Co)

if rank_Co ~= n
    error('The system is not completely controllable.')
end

% State-feedback gain
K = place(A,B,controller_poles_desired);

% Full-state closed-loop matrix
A_controller = A-B*K;

% Reference prefilter
N_r = -1/(C_phi*(A_controller\B));

fprintf('\nState-feedback gain K:\n')
disp(K)

fprintf('Reference prefilter:\n')
fprintf('N_r = %.8f\n',N_r)

fprintf('\nActual controller poles:\n')
disp(eig(A_controller))

%% ============================================================
% 2. Observability from the p measurement
%% ============================================================

Ob_p = obsv(A,C_p);
rank_Ob_p = rank(Ob_p);

fprintf('\nObservability rank from p = %d\n',rank_Ob_p)

if rank_Ob_p ~= n
    error('The system is not completely observable from p.')
end

%% ============================================================
% 3. Observer design
%% ============================================================

% Observer poles
observer_poles_desired = [-5 -6 -7 -8 -9];

% Observer gain
L = place(A',C_p',observer_poles_desired)';

A_observer = A-L*C_p;

fprintf('\nObserver gain L:\n')
disp(L)

fprintf('Actual observer poles:\n')
disp(eig(A_observer))

%% ============================================================
% 4. Augmented observer-based closed-loop system
%
% x_aug = [x; x_hat]
%
% Control law:
% delta_c = -K*x_hat + N_r*phi_c
%% ============================================================

A_aug = [ ...
    A,                  -B*K;
    L*C_p, A-B*K-L*C_p];

B_aug = [ ...
    B*N_r;
    B*N_r];

% Output all plant and observer states
C_aug_states = eye(2*n);
D_aug_states = zeros(2*n,1);

sys_aug_states = ss( ...
    A_aug, ...
    B_aug, ...
    C_aug_states, ...
    D_aug_states);

% Output phi only
C_aug_phi = [C_phi zeros(1,n)];

sys_aug_phi = ss( ...
    A_aug, ...
    B_aug, ...
    C_aug_phi, ...
    0);

fprintf('\nAugmented-system poles:\n')
disp(eig(A_aug))

%% ============================================================
% 5. Command-tracking simulation
%% ============================================================

phi_command_deg = 30;
phi_command_rad = deg2rad(phi_command_deg);

t = (0:0.001:10)';
r_command = phi_command_rad*ones(size(t));

% Nominal initial conditions
x0 = zeros(n,1);
xhat0 = zeros(n,1);

x_aug_response = lsim( ...
    sys_aug_states, ...
    r_command, ...
    t, ...
    [x0; xhat0]);

% Separate true and estimated states
x = x_aug_response(:,1:n);
xhat = x_aug_response(:,n+1:2*n);

% Plant outputs
phi_observer_deg = rad2deg(x(:,4));
delta_a_deg = rad2deg(x(:,5));

% Control command
delta_c_rad = -xhat*K.' + N_r*r_command;
delta_c_deg = rad2deg(delta_c_rad);

%% ============================================================
% 6. Full-state feedback response for comparison
%% ============================================================

sys_full_states = ss( ...
    A_controller, ...
    B*N_r, ...
    eye(n), ...
    zeros(n,1));

x_full = lsim( ...
    sys_full_states, ...
    r_command, ...
    t, ...
    zeros(n,1));

phi_full_deg = rad2deg(x_full(:,4));

%% ============================================================
% 7. Performance calculations
%% ============================================================

phi_ss_deg = ...
    phi_command_deg*real(dcgain(sys_aug_phi));

steady_state_error_deg = ...
    phi_command_deg-phi_ss_deg;

info = stepinfo( ...
    phi_observer_deg, ...
    t, ...
    phi_command_deg, ...
    'SettlingTimeThreshold',0.02);

max_delta_a = max(abs(delta_a_deg));
max_delta_c = max(abs(delta_c_deg));

fprintf('\n============================================\n')
fprintf('OBSERVER-BASED PERFORMANCE USING p\n')
fprintf('============================================\n')

fprintf('Steady-state phi = %.8f deg\n',phi_ss_deg)

fprintf('Steady-state error = %.8e deg\n', ...
    steady_state_error_deg)

fprintf('Rise time = %.4f s\n', ...
    info.RiseTime)

fprintf('Settling time = %.4f s\n', ...
    info.SettlingTime)

fprintf('Overshoot = %.4f %%\n', ...
    info.Overshoot)

fprintf('Max |delta_a| = %.4f deg\n', ...
    max_delta_a)

fprintf('Max |delta_c| = %.4f deg\n', ...
    max_delta_c)

%% ============================================================
% 8. Observer convergence test
%
% A nonzero initial estimation error is introduced.
%% ============================================================

r_zero = zeros(size(t));

% True initial state
% beta, phi and delta_a are in deg
% p and r are in deg/s
x0_test = [ ...
    deg2rad(0.5);
    deg2rad(0.3);
    deg2rad(0.3);
    deg2rad(1.0);
    deg2rad(0.2)];

% Observer starts from zero
xhat0_test = zeros(n,1);

x_aug_test = lsim( ...
    sys_aug_states, ...
    r_zero, ...
    t, ...
    [x0_test; xhat0_test]);

x_test = x_aug_test(:,1:n);
xhat_test = x_aug_test(:,n+1:2*n);

estimation_error = x_test-xhat_test;

%% ============================================================
% 9. Pole information in the command window
%% ============================================================

controller_poles_actual = eig(A_controller);
observer_poles_actual = eig(A_observer);

% Pole-only systems for pzplot
sys_controller_poles = zpk( ...
    [], ...
    controller_poles_actual, ...
    1);

sys_observer_poles = zpk( ...
    [], ...
    observer_poles_actual, ...
    1);

sys_controller_poles.Name = 'Controller poles';
sys_observer_poles.Name = 'Observer poles';

fprintf('\nController-pole damping data:\n')
damp(sys_controller_poles)

fprintf('\nObserver-pole damping data:\n')
damp(sys_observer_poles)

%% ============================================================
% Figure 1
% Controller and observer poles on one control-system grid
%% ============================================================

figure('Name','Controller and Observer Pole Map')

% pzplot creates proper control-system Data Tips.
% Clicking a pole shows pole location, damping and natural frequency.
h_poles = pzplot( ...
    sys_controller_poles, ...
    sys_observer_poles);

hold on

% IMPORTANT:
% sgrid accepts damping ratios strictly between 0 and 1.
% Do not insert zeta = 1.
%
% The real observer poles are still plotted, and pzplot reports
% their damping ratio as zeta = 1 in the Data Tip.

zeta_grid = [0.5 0.6 0.7];

wn_grid = [ ...
    2 ...
    2.5 ...
    3 ...
    5 ...
    6 ...
    7 ...
    8 ...
    9];

sgrid(zeta_grid,wn_grid)

title('Controller and Observer Poles')

xlabel('Real Axis [1/s]')
ylabel('Imaginary Axis [rad/s]')

legend( ...
    'Controller poles', ...
    'Observer poles', ...
    'Location','northwest')

xlim([-10 0.5])
ylim([-3.5 3.5])

%% ============================================================
% Figure 2
% Full-state versus observer-based response
%% ============================================================

figure('Name','Roll Response Comparison')

plot( ...
    t, ...
    phi_full_deg, ...
    '--', ...
    'LineWidth',1.6)

hold on

plot( ...
    t, ...
    phi_observer_deg, ...
    'LineWidth',1.8)

yline( ...
    phi_command_deg, ...
    'k:', ...
    'Command = 30 deg', ...
    'HandleVisibility','off')

grid on

xlabel('Time [s]')
ylabel('\phi [deg]')

title('Full-State and p-Observer Roll Responses')

legend( ...
    'Full-state feedback', ...
    'Observer based on p', ...
    'Location','best')

xlim([0 6])

%% ============================================================
% Figure 3
% Angular-state estimation errors
%% ============================================================

figure('Name','Angular-State Estimation Errors')

plot( ...
    t, ...
    rad2deg(estimation_error(:,1)), ...
    'LineWidth',1.4)

hold on

plot( ...
    t, ...
    rad2deg(estimation_error(:,4)), ...
    'LineWidth',1.4)

plot( ...
    t, ...
    rad2deg(estimation_error(:,5)), ...
    'LineWidth',1.4)

grid on

xlabel('Time [s]')
ylabel('Estimation error [deg]')

title('Angular-State Estimation Errors')

legend( ...
    'e_\beta', ...
    'e_\phi', ...
    'e_{\delta_a}', ...
    'Location','best')

xlim([0 2])

%% ============================================================
% Figure 4
% Angular-rate estimation errors
%% ============================================================

figure('Name','Angular-Rate Estimation Errors')

plot( ...
    t, ...
    rad2deg(estimation_error(:,2)), ...
    'LineWidth',1.4)

hold on

plot( ...
    t, ...
    rad2deg(estimation_error(:,3)), ...
    'LineWidth',1.4)

grid on

xlabel('Time [s]')
ylabel('Estimation error [deg/s]')

title('Angular-Rate Estimation Errors')

legend( ...
    'e_p', ...
    'e_r', ...
    'Location','best')

xlim([0 2])

%% ============================================================
% Figure 5
% Aileron deflection and controller command
%% ============================================================

figure('Name','Control Effort')

plot( ...
    t, ...
    delta_a_deg, ...
    'LineWidth',1.8)

hold on

plot( ...
    t, ...
    delta_c_deg, ...
    '--', ...
    'LineWidth',1.5)

yline( ...
    5, ...
    'k:', ...
    '+5 deg limit', ...
    'HandleVisibility','off')

yline( ...
    -5, ...
    'k:', ...
    '-5 deg limit', ...
    'HandleVisibility','off')

grid on

xlabel('Time [s]')
ylabel('Angle [deg]')

title('Aileron Deflection and Control Command')

legend( ...
    '\delta_a(t)', ...
    '\delta_c(t)', ...
    'Location','best')

xlim([0 6])