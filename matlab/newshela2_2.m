%% Q2(b) - Inner roll-rate loop and outer roll-angle loop
% Interactive Root-Locus design and complete verification

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

tol = 1e-7;

%% Open-loop poles

poles_OL = eig(A);

disp('Open-loop poles:')
disp(poles_OL)

%% ============================================================
% Part 1 - Interactive selection of Kp
%
% Control law for the inner loop:
%
% delta_c = v - Kp*p
%% ============================================================

G_p = minreal(tf(ss(A,B,C_p,0)),1e-7);

disp('Inner-loop plant G_p(s) = p/delta_c:')
G_p

fprintf('\nOpening the Root Locus for the inner p loop.\n')
fprintf('Select the desired point and read the gain from the app.\n')
fprintf('The expected design value is approximately Kp = 0.01.\n\n')

controlSystemDesigner("rlocus",G_p);
drawnow

Kp = input('Enter the Kp selected in Control System Designer: ');

if Kp < 0
    error(['For the control law delta_c = v - Kp*p, ' ...
           'Kp must be entered as a positive value.'])
end

%% Close the inner p loop

A_inner = A-B*Kp*C_p;

poles_inner = eig(A_inner);

disp('Poles after closing the inner p loop:')
disp(poles_inner)

%% Identify Dutch Roll after inner-loop closure

complex_OL = poles_OL(imag(poles_OL) > tol);
complex_inner = poles_inner(imag(poles_inner) > tol);

[~,idx_DR_OL] = max(abs(imag(complex_OL)));
[~,idx_DR_inner] = max(abs(imag(complex_inner)));

p_DR_OL = complex_OL(idx_DR_OL);
p_DR_inner = complex_inner(idx_DR_inner);

zeta_DR_OL = -real(p_DR_OL)/abs(p_DR_OL);
zeta_DR_inner = -real(p_DR_inner)/abs(p_DR_inner);

fprintf('\nDutch-Roll comparison:\n')
fprintf('Open loop:  %.4f %+.4fj, zeta = %.4f\n', ...
    real(p_DR_OL),imag(p_DR_OL),zeta_DR_OL)

fprintf('Inner loop: %.4f %+.4fj, zeta = %.4f\n', ...
    real(p_DR_inner),imag(p_DR_inner),zeta_DR_inner)

%% Identify Spiral pole

real_OL = poles_OL(abs(imag(poles_OL)) < tol);
real_inner = poles_inner(abs(imag(poles_inner)) < tol);

p_spiral_OL = max(real(real_OL));
p_spiral_inner = max(real(real_inner));

fprintf('\nSpiral-mode comparison:\n')
fprintf('Open loop:  %.4f\n',p_spiral_OL)
fprintf('Inner loop: %.4f\n',p_spiral_inner)

%% Figure 1 - Effect of the inner p loop

figure

plot(real(poles_OL),imag(poles_OL),'kx', ...
    'MarkerSize',10, ...
    'LineWidth',1.8)

hold on

plot(real(poles_inner),imag(poles_inner),'ro', ...
    'MarkerSize',8, ...
    'LineWidth',1.6)

xline(0,'k--')
yline(0,'k--')

grid on

xlabel('Real Axis [1/sec]')
ylabel('Imaginary Axis [rad/sec]')
title(sprintf('Effect of Inner Roll-Rate Feedback, K_p = %.5f',Kp))

legend('Open-Loop Poles','Poles after p Feedback', ...
    'Location','best')

%% ============================================================
% Part 2 - Interactive selection of Kphi
%% ============================================================

% Outer-loop plant from v to phi after closing the p loop
G_outer = minreal(tf(ss(A_inner,B,C_phi,0)),1e-7);

disp('Outer-loop plant G_outer(s) = phi/v:')
G_outer

fprintf('\nOpening the Root Locus for the outer phi loop.\n')
fprintf('Add the damping-ratio requirement yourself.\n')
fprintf('The expected selection is approximately Kphi = 0.07408.\n\n')

controlSystemDesigner("rlocus",G_outer);
drawnow

Kphi = input('Enter the Kphi selected in Control System Designer: ');

zeta_requirement = input( ...
    'Enter the damping-ratio requirement used in the app: ');

if Kphi <= 0
    error('Kphi must be positive.')
end

%% Complete closed-loop model

% Complete control law:
%
% delta_c = Kphi*(phi_c-phi) - Kp*p

A_cl = A-B*Kp*C_p-B*Kphi*C_phi;
B_cl = B*Kphi;

poles_CL = eig(A_cl);

fprintf('\nSelected controller:\n')
fprintf('Kp   = %.8f\n',Kp)
fprintf('Kphi = %.8f\n',Kphi)

disp('Complete closed-loop poles:')
disp(poles_CL)

%% Dominant closed-loop pair

complex_CL = poles_CL(imag(poles_CL) > tol);

if isempty(complex_CL)
    error('No complex closed-loop pole pair was found.')
end

% Dominant complex pair = closest to the imaginary axis
[~,idx_dom] = max(real(complex_CL));

p_dom = complex_CL(idx_dom);

wn_dom = abs(p_dom);
zeta_dom = -real(p_dom)/wn_dom;

fprintf('\nDominant closed-loop pair:\n')
fprintf('s = %.5f %+.5fj\n',real(p_dom),imag(p_dom))
fprintf('wn = %.5f rad/sec\n',wn_dom)
fprintf('zeta = %.5f\n',zeta_dom)

if zeta_dom >= zeta_requirement
    fprintf('The selected damping-ratio requirement is satisfied.\n')
else
    warning('The selected design does not satisfy the damping requirement.')
end

%% Figure 2 - Clean outer-loop Root Locus

figure

rlocus(G_outer)
hold on
grid on

% Display the same constraint entered in the design app
sgrid(zeta_requirement,[])

% Mark the poles produced by the selected Kphi
plot(real(poles_CL),imag(poles_CL),'rs', ...
    'MarkerSize',9, ...
    'LineWidth',1.8)

design_label = { ...
    sprintf('K_p = %.5f',Kp), ...
    sprintf('K_\\phi = %.5f',Kphi), ...
    sprintf('\\zeta = %.3f',zeta_dom), ...
    sprintf('\\omega_n = %.3f rad/s',wn_dom)};

text(real(p_dom)-2.2,imag(p_dom)+1.0, ...
    design_label, ...
    'FontSize',10, ...
    'Interpreter','tex', ...
    'BackgroundColor','white', ...
    'EdgeColor',[0.5 0.5 0.5], ...
    'Margin',5)

title('Outer-Loop Root Locus and Selected Design')
xlabel('Real Axis [1/sec]')
ylabel('Imaginary Axis [rad/sec]')

xlim([-8 1])
ylim([-10 10])

%% Closed-loop simulation

C_out = [C_phi;
         C_p;
         C_da];

D_out = zeros(3,1);

sys_CL = ss(A_cl,B_cl,C_out,D_out);

phi_command_deg = 30;
phi_command_rad = deg2rad(phi_command_deg);

t = (0:0.001:15)';
r = phi_command_rad*ones(size(t));

[y,t] = lsim(sys_CL,r,t);

phi_rad = y(:,1);
p_rad_sec = y(:,2);
delta_a_rad = y(:,3);

phi_deg = rad2deg(phi_rad);
p_deg_sec = rad2deg(p_rad_sec);
delta_a_deg = rad2deg(delta_a_rad);

%% Controller command

delta_c_rad = Kphi*(r-phi_rad)-Kp*p_rad_sec;
delta_c_deg = rad2deg(delta_c_rad);

%% Performance calculations

sys_phi_CL = ss(A_cl,B_cl,C_phi,0);

phi_final_deg = phi_command_deg*dcgain(sys_phi_CL);

e_ss_deg = phi_command_deg-phi_final_deg;
e_ss_percent = 100*e_ss_deg/phi_command_deg;

tracking_error_deg = phi_command_deg-phi_deg;

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

%% Figure 3 - Roll-angle response

figure

plot(t,phi_deg,'LineWidth',1.6)
hold on

yline(phi_command_deg,'--','Command = 30 deg', ...
    'LineWidth',1.2)

yline(phi_final_deg,':','Steady-state value', ...
    'LineWidth',1.2)

grid on

xlabel('Time [sec]')
ylabel('\phi [deg]')
title('Roll-Angle Response with Roll-Rate Feedback')

legend('\phi(t)','Command','Steady-State Value', ...
    'Location','best')

%% Figure 4 - Tracking error

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

%% Figure 5 - Aileron and servo command

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