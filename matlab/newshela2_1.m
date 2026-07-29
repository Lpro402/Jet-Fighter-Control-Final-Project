%% Q2(a) - Proportional controller after interactive Root-Locus design

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
C_p   = [0 1 0 0 0];
C_da  = [0 0 0 0 1];

%% Plant transfer function: phi / delta_c

G_phi = minreal(tf(ss(A,B,C_phi,0)),1e-7);

disp('Plant transfer function G_phi_delta_c(s):')
G_phi

%% Gain selected manually in Control System Designer

K = 0.047953;

% Damping-ratio requirement selected in the design tool
zeta_requirement = 0.5;

%% Closed-loop transfer function

T_phi = minreal(feedback(K*G_phi,1),1e-7);

poles_ol = pole(G_phi);
poles_cl = pole(T_phi);

disp('Selected gain:')
fprintf('K = %.6f\n',K)

disp('Closed-loop poles:')
disp(poles_cl)

%% Identify the dominant closed-loop pole pair

tol = 1e-7;

complex_cl = poles_cl(imag(poles_cl) > tol);

if isempty(complex_cl)
    error('No complex closed-loop pole pair was found.')
end

% Dominant complex pole: closest complex pole to imaginary axis
[~,idx_dom] = max(real(complex_cl));
p_dom = complex_cl(idx_dom);

wn_dom   = abs(p_dom);
zeta_dom = -real(p_dom)/wn_dom;

fprintf('\nDominant closed-loop pole pair:\n')
fprintf('s = %.4f %+.4fj\n',real(p_dom),imag(p_dom))
fprintf('wn = %.4f rad/sec\n',wn_dom)
fprintf('zeta = %.4f\n',zeta_dom)

if zeta_dom >= zeta_requirement
    fprintf('The damping-ratio requirement is satisfied.\n')
else
    warning('The selected gain does not satisfy zeta >= %.2f.', ...
        zeta_requirement)
end

%% Dutch-Roll comparison

complex_ol = poles_ol(imag(poles_ol) > tol);

% Dutch Roll: complex pair with the largest imaginary part
[~,idx_DR_ol] = max(abs(imag(complex_ol)));
[~,idx_DR_cl] = max(abs(imag(complex_cl)));

p_DR_ol = complex_ol(idx_DR_ol);
p_DR_cl = complex_cl(idx_DR_cl);

zeta_DR_ol = -real(p_DR_ol)/abs(p_DR_ol);
zeta_DR_cl = -real(p_DR_cl)/abs(p_DR_cl);

fprintf('\nDutch-Roll comparison:\n')
fprintf('Open loop:   s = %.4f %+.4fj, zeta = %.4f\n', ...
    real(p_DR_ol),imag(p_DR_ol),zeta_DR_ol)

fprintf('Closed loop: s = %.4f %+.4fj, zeta = %.4f\n', ...
    real(p_DR_cl),imag(p_DR_cl),zeta_DR_cl)

%% Root-Locus figure with selected design

figure

rlocus(G_phi)
hold on
grid on

% Plot damping-ratio requirement
sgrid(zeta_requirement,[])

% Mark selected closed-loop poles
plot(real(poles_cl),imag(poles_cl),'rs', ...
    'MarkerSize',9, ...
    'LineWidth',1.8)

% Multiline label with valid TeX syntax
design_label = { ...
    sprintf('K = %.6f',K), ...
    sprintf('\\zeta = %.3f',zeta_dom), ...
    sprintf('\\omega_n = %.3f rad/s',wn_dom)};

text(real(p_dom)-2.2,imag(p_dom)+1.0, ...
    design_label, ...
    'FontSize',10, ...
    'Interpreter','tex', ...
    'BackgroundColor','white', ...
    'EdgeColor',[0.5 0.5 0.5], ...
    'Margin',5)

title('Root Locus and Selected Proportional-Control Design')
xlabel('Real Axis [1/sec]')
ylabel('Imaginary Axis [rad/sec]')

xlim([-8 1])
ylim([-10 10])

%% Closed-loop state-space model

% Control law:
% delta_c = K(phi_c - phi)

A_cl = A-B*K*C_phi;
B_cl = B*K;

C_out = [C_phi;
         C_p;
         C_da];

D_out = zeros(3,1);

sys_cl = ss(A_cl,B_cl,C_out,D_out);

%% Simulation for a 30-degree roll-angle command

phi_cmd_deg = 30;
phi_cmd_rad = deg2rad(phi_cmd_deg);

t = (0:0.001:12)';
r = phi_cmd_rad*ones(size(t));

[y,t] = lsim(sys_cl,r,t);

phi_rad    = y(:,1);
p_rad_sec  = y(:,2);
delta_a_rad = y(:,3);

phi_deg     = rad2deg(phi_rad);
p_deg_sec   = rad2deg(p_rad_sec);
delta_a_deg = rad2deg(delta_a_rad);

%% Servo command delta_c

delta_c_rad = K*(r-phi_rad);
delta_c_deg = rad2deg(delta_c_rad);

%% Tracking error

tracking_error_deg = phi_cmd_deg-phi_deg;

%% Steady-state and transient performance

sys_phi_cl = ss(A_cl,B_cl,C_phi,0);

phi_final_deg = phi_cmd_deg*dcgain(sys_phi_cl);

e_ss_deg = phi_cmd_deg-phi_final_deg;
e_ss_percent = 100*e_ss_deg/phi_cmd_deg;

response_info = stepinfo(phi_deg,t,phi_final_deg);

max_delta_a = max(abs(delta_a_deg));
max_delta_c = max(abs(delta_c_deg));

fprintf('\nStep-response performance:\n')
fprintf('Final phi = %.4f deg\n',phi_final_deg)
fprintf('Steady-state error = %.4f deg\n',e_ss_deg)
fprintf('Steady-state error = %.4f %%\n',e_ss_percent)
fprintf('Rise time = %.4f sec\n',response_info.RiseTime)
fprintf('Settling time = %.4f sec\n',response_info.SettlingTime)
fprintf('Overshoot = %.4f %%\n',response_info.Overshoot)
fprintf('Max |delta_a| = %.4f deg\n',max_delta_a)
fprintf('Max |delta_c| = %.4f deg\n',max_delta_c)

%% Roll-angle response

figure

plot(t,phi_deg,'LineWidth',1.6)
hold on

yline(phi_cmd_deg,'--','Command = 30 deg', ...
    'LineWidth',1.2)

yline(phi_final_deg,':','Steady-state value', ...
    'LineWidth',1.2)

grid on
xlabel('Time [sec]')
ylabel('\phi [deg]')
title('Roll-Angle Response with Proportional Control')

legend('\phi(t)','Command','Steady-State Value', ...
    'Location','best')

%% Tracking-error response

figure

plot(t,tracking_error_deg,'LineWidth',1.6)
hold on

yline(e_ss_deg,'--', ...
    sprintf('e_{ss} = %.3f deg',e_ss_deg), ...
    'LineWidth',1.2)

grid on
xlabel('Time [sec]')
ylabel('e(t) [deg]')
title('Roll-Angle Tracking Error')

legend('e(t)=\phi_c-\phi','Steady-State Error', ...
    'Location','best')

%% Aileron deflection and servo command

figure

plot(t,delta_a_deg,'LineWidth',1.6)
hold on

plot(t,delta_c_deg,'--','LineWidth',1.4)

yline(5,':','+5 deg limit','LineWidth',1.2)
yline(-5,':','-5 deg limit','LineWidth',1.2)

grid on
xlabel('Time [sec]')
ylabel('Angle [deg]')
title('Aileron Deflection and Servo Command')

legend('\delta_a(t)','\delta_c(t)','Aileron Limits', ...
    'Location','best')