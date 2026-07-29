%% Question 3(b) - Observer-Based State Feedback
% Available measurement: roll angle phi only

clear
close all
clc

%% Aircraft model

A = [ ...
    -0.575     0       -1       0.0536   -0.078;
    -300      -3.03     2       0        64.4;
      68       0.045   -2.4     0         5;
       0       1        0       0         0;
       0       0        0       0        -5];

B = [0; 0; 0; 0; 5];

C_phi = [0 0 0 1 0];

n = size(A,1);

%% ============================================================
% Controller design from Question 3(a)
%% ============================================================

controller_poles = [ ...
    -1.8 + 2.4i;
    -1.8 - 2.4i;
    -2.1 + 2.14242853i;
    -2.1 - 2.14242853i;
    -2.05];

K = place(A,B,controller_poles);

A_cl = A-B*K;

N_r = -1/(C_phi*(A_cl\B));

fprintf('State-feedback gain K:\n')
disp(K)

fprintf('Reference prefilter:\n')
fprintf('N_r = %.8f\n\n',N_r)

fprintf('Controller poles:\n')
disp(eig(A_cl))

%% ============================================================
% Observability check
%% ============================================================

Ob = obsv(A,C_phi);
rank_Ob = rank(Ob);

fprintf('Observability rank = %d\n',rank_Ob)

if rank_Ob ~= n
    error('The system is not completely observable from phi.')
end

%% ============================================================
% Observer design
%% ============================================================

observer_poles = [-6 -7 -8 -9 -10];

L = place(A',C_phi',observer_poles)';

fprintf('\nObserver gain L:\n')
disp(L)

fprintf('Actual observer poles:\n')
disp(eig(A-L*C_phi))

%% ============================================================
% Augmented observer-based system
%
% x_aug = [x; x_hat]
%
% delta_c = -K*x_hat + N_r*phi_c
%% ============================================================

A_aug = [ ...
    A,                  -B*K;
    L*C_phi, A-B*K-L*C_phi];

B_aug = [ ...
    B*N_r;
    B*N_r];

C_aug = eye(2*n);
D_aug = zeros(2*n,1);

sys_aug = ss(A_aug,B_aug,C_aug,D_aug);

%% Verify separation principle

augmented_poles = eig(A_aug);

fprintf('\nAugmented-system poles:\n')
disp(augmented_poles)

%% ============================================================
% Nominal command-tracking simulation
%% ============================================================

phi_command_deg = 30;
phi_command_rad = deg2rad(phi_command_deg);

t = (0:0.001:10)';
r = phi_command_rad*ones(size(t));

% Same initial condition for plant and observer
x0_nominal = zeros(5,1);
xhat0_nominal = zeros(5,1);

xaug0_nominal = [x0_nominal; xhat0_nominal];

x_aug_nominal = lsim( ...
    sys_aug,r,t,xaug0_nominal);

x_nominal = x_aug_nominal(:,1:5);
xhat_nominal = x_aug_nominal(:,6:10);

phi_observer_deg = rad2deg(x_nominal(:,4));
delta_a_deg = rad2deg(x_nominal(:,5));

delta_c_rad = ...
    -xhat_nominal*K.' + N_r*r;

delta_c_deg = rad2deg(delta_c_rad);

%% Full-state feedback response for comparison

sys_full_states = ss( ...
    A_cl, ...
    B*N_r, ...
    eye(5), ...
    zeros(5,1));

x_full = lsim( ...
    sys_full_states,r,t,zeros(5,1));

phi_full_deg = rad2deg(x_full(:,4));

%% Performance calculations

sys_phi_observer = ss( ...
    A_aug, ...
    B_aug, ...
    [C_phi zeros(1,5)], ...
    0);

phi_final_deg = ...
    phi_command_deg*dcgain(sys_phi_observer);

e_ss_deg = phi_command_deg-phi_final_deg;
e_ss_percent = 100*e_ss_deg/phi_command_deg;

response_info = stepinfo( ...
    phi_observer_deg, ...
    t, ...
    phi_final_deg, ...
    'SettlingTimeThreshold',0.02);

max_delta_a = max(abs(delta_a_deg));
max_delta_c = max(abs(delta_c_deg));

fprintf('\nNominal observer-based performance:\n')
fprintf('Final phi = %.6f deg\n',phi_final_deg)
fprintf('Steady-state error = %.8f deg\n',e_ss_deg)
fprintf('Steady-state error = %.8f %%\n',e_ss_percent)
fprintf('Rise time = %.4f s\n',response_info.RiseTime)
fprintf('Settling time = %.4f s\n', ...
    response_info.SettlingTime)
fprintf('Overshoot = %.4f %%\n', ...
    response_info.Overshoot)
fprintf('Max |delta_a| = %.4f deg\n',max_delta_a)
fprintf('Max |delta_c| = %.4f deg\n',max_delta_c)

%% ============================================================
% Observer-convergence test
%
% A small initial estimation error is introduced so that
% the convergence of the observer can be seen.
%% ============================================================

r_zero = zeros(size(t));

x0_test = deg2rad([ ...
    0.5;
    0.3;
    0.3;
    1.0;
    0.2]);

xhat0_test = zeros(5,1);

xaug0_test = [x0_test; xhat0_test];

x_aug_test = lsim( ...
    sys_aug,r_zero,t,xaug0_test);

x_test = x_aug_test(:,1:5);
xhat_test = x_aug_test(:,6:10);

estimation_error = x_test-xhat_test;

%% ============================================================
% Figure 1 - Controller and observer poles
%% ============================================================

figure('Name','Controller and Observer Poles')

hold on
grid on
axis equal

%% Controller design region

zeta_min = 0.5;
zeta_max = 0.7;

wn_min = 2;
wn_max = 3;

theta = linspace( ...
    acos(zeta_max), ...
    acos(zeta_min), ...
    300);

x_outer = -wn_max*cos(theta);
y_outer =  wn_max*sin(theta);

x_inner = -wn_min*cos(fliplr(theta));
y_inner =  wn_min*sin(fliplr(theta));

x_region = [x_outer x_inner];
y_region = [y_outer y_inner];

fill(x_region,y_region,[0.85 0.92 1.00], ...
    'FaceAlpha',0.5, ...
    'EdgeColor','none', ...
    'DisplayName','Controller-pole design region');

fill(x_region,-y_region,[0.85 0.92 1.00], ...
    'FaceAlpha',0.5, ...
    'EdgeColor','none', ...
    'HandleVisibility','off');

%% Controller poles

h_controller = scatter( ...
    real(controller_poles), ...
    imag(controller_poles), ...
    100, ...
    'x', ...
    'LineWidth',2, ...
    'DisplayName','Controller poles');

wn_controller = abs(controller_poles);
zeta_controller = ...
    -real(controller_poles)./wn_controller;

h_controller.DataTipTemplate.DataTipRows(end+1) = ...
    dataTipTextRow('\omega_n',wn_controller);

h_controller.DataTipTemplate.DataTipRows(end+1) = ...
    dataTipTextRow('\zeta',zeta_controller);

%% Observer poles

h_observer = scatter( ...
    real(observer_poles), ...
    imag(observer_poles), ...
    80, ...
    'o', ...
    'LineWidth',1.8, ...
    'DisplayName','Observer poles');

wn_observer = abs(observer_poles);
zeta_observer = ones(size(observer_poles));

h_observer.DataTipTemplate.DataTipRows(end+1) = ...
    dataTipTextRow('\omega_n',wn_observer);

h_observer.DataTipTemplate.DataTipRows(end+1) = ...
    dataTipTextRow('\zeta',zeta_observer);

xline(0,'k--','HandleVisibility','off')
yline(0,'k--','HandleVisibility','off')

xlabel('Real Axis [1/s]')
ylabel('Imaginary Axis [rad/s]')

title('Controller and Observer Pole Locations')

legend('Location','best')

xlim([-11 0.5])
ylim([-3.5 3.5])

%% ============================================================
% Figure 2 - Full-state and observer-based responses
%% ============================================================

figure('Name','Observer-Based Response')

plot(t,phi_full_deg,'--','LineWidth',1.5)
hold on

plot(t,phi_observer_deg,'LineWidth',1.8)

yline(phi_command_deg,'k:', ...
    'Command = 30 deg', ...
    'HandleVisibility','off');

grid on

xlabel('Time [s]')
ylabel('\phi [deg]')

title('Full-State and Observer-Based Roll Responses')

legend( ...
    'Full-state feedback', ...
    'Observer-based feedback', ...
    'Location','best')

xlim([0 6])

%% ============================================================
% Figure 3 - Estimation errors
%% ============================================================

figure('Name','Observer Estimation Errors')

tiledlayout(2,1)

nexttile

plot(t,rad2deg(estimation_error(:,1)), ...
    'LineWidth',1.4)
hold on

plot(t,rad2deg(estimation_error(:,4)), ...
    'LineWidth',1.4)

plot(t,rad2deg(estimation_error(:,5)), ...
    'LineWidth',1.4)

grid on

xlabel('Time [s]')
ylabel('Error [deg]')

title('Angular-State Estimation Errors')

legend( ...
    'e_\beta', ...
    'e_\phi', ...
    'e_{\delta_a}', ...
    'Location','best')

xlim([0 2])

nexttile

plot(t,rad2deg(estimation_error(:,2)), ...
    'LineWidth',1.4)
hold on

plot(t,rad2deg(estimation_error(:,3)), ...
    'LineWidth',1.4)

grid on

xlabel('Time [s]')
ylabel('Error [deg/s]')

title('Angular-Rate Estimation Errors')

legend( ...
    'e_p', ...
    'e_r', ...
    'Location','best')

xlim([0 2])

%% ============================================================
% Figure 4 - Aileron and control command
%% ============================================================

figure('Name','Observer-Based Control Effort')

plot(t,delta_a_deg,'LineWidth',1.8)
hold on

plot(t,delta_c_deg,'--','LineWidth',1.5)

yline(5,'k:', ...
    '+5 deg limit', ...
    'HandleVisibility','off');

yline(-5,'k:', ...
    '-5 deg limit', ...
    'HandleVisibility','off');

grid on

xlabel('Time [s]')
ylabel('Angle [deg]')

title('Aileron Deflection and Control Command')

legend( ...
    '\delta_a(t)', ...
    '\delta_c(t)', ...
    'Location','best')

xlim([0 6])