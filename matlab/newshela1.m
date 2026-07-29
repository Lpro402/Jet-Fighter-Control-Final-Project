%% Q1 - Five-state model and modal analysis

clear;
clc;
close all;

%% Five-state model

A = [ ...
    -0.575,    0,       -1,      0.0536,  -0.078;
    -300,     -3.03,     2,      0,        64.4;
      68,      0.045,   -2.4,    0,         5;
       0,      1,        0,      0,         0;
       0,      0,        0,      0,        -5];

B = [0; 0; 0; 0; 5];

C_phi = [0 0 0 1 0];
C_p   = [0 1 0 0 0];

C = [C_phi;
     C_p];

D = [0; 0];

sys = ss(A,B,C,D);

%% Open-loop poles

poles_ol = eig(A);

disp('Open-loop poles:')
disp(poles_ol)

%% Separate complex and real poles

tol = 1e-8;

complex_poles = poles_ol(abs(imag(poles_ol)) > tol);
real_poles    = real(poles_ol(abs(imag(poles_ol)) <= tol));

% Take one pole from the conjugate Dutch-Roll pair
p_DR = complex_poles(imag(complex_poles) > 0);

% Sort real poles from slowest to fastest
real_poles = sort(real_poles,'descend');

p_spiral = real_poles(1);
p_roll   = real_poles(2);
p_servo  = real_poles(3);

%% Modal properties

wn_DR   = abs(p_DR);
zeta_DR = -real(p_DR)/wn_DR;
tau_DR  = 1/abs(real(p_DR));

wn_spiral  = abs(p_spiral);
tau_spiral = 1/abs(p_spiral);

wn_roll  = abs(p_roll);
tau_roll = 1/abs(p_roll);

wn_servo  = abs(p_servo);
tau_servo = 1/abs(p_servo);

%% Results table
% Use cell arrays for text of different lengths

Mode = { ...
    'Dutch Roll';
    'Spiral';
    'Roll';
    'Aileron Servo'};

Pole = { ...
    sprintf('%.4f %+.4fj',real(p_DR),imag(p_DR));
    sprintf('%.4f',p_spiral);
    sprintf('%.4f',p_roll);
    sprintf('%.4f',p_servo)};

wn = [ ...
    wn_DR;
    wn_spiral;
    wn_roll;
    wn_servo];

% Damping ratio is meaningful here for the complex pair.
% The real modes are first-order modes.
zeta = [ ...
    zeta_DR;
    NaN;
    NaN;
    NaN];

tau = [ ...
    tau_DR;
    tau_spiral;
    tau_roll;
    tau_servo];

ModalTable = table(Mode,Pole,wn,zeta,tau, ...
    'VariableNames',{'Mode','Pole','wn_rad_sec','Zeta','Tau_sec'});

disp('Modal properties:')
disp(ModalTable)

%% Transfer function from delta_c to phi

sys_phi = ss(A,B,C_phi,0);
G_phi = minreal(tf(sys_phi),1e-7);

disp('Transfer function G_phi_delta_c(s):')
G_phi

zeros_phi = zero(G_phi);

disp('Zeros of G_phi_delta_c(s):')
disp(zeros_phi)

%% Stability check

if all(real(poles_ol) < 0)
    disp('The open-loop system is asymptotically stable.');
else
    disp('The open-loop system is not asymptotically stable.');
end

%% Normal pole-zero map, similar to the tutorials

figure;
pzmap(G_phi);
grid on;
title('Pole-Zero Map of G_{\phi\delta_c}(s)');
xlabel('Real Axis [1/sec]');
ylabel('Imaginary Axis [rad/sec]');

%% Optional: pole map with mode labels

figure;
plot(real(poles_ol),imag(poles_ol),'x', ...
    'MarkerSize',10,'LineWidth',2);
hold on;

plot(real(zeros_phi),imag(zeros_phi),'o', ...
    'MarkerSize',9,'LineWidth',2);

xline(0,'k--');
yline(0,'k--');
grid on;

text(real(p_DR)+0.15, imag(p_DR), 'Dutch Roll');
text(real(conj(p_DR))+0.15, imag(conj(p_DR)), 'Dutch Roll');

text(p_spiral,0.35,'Spiral', ...
    'HorizontalAlignment','center');

text(p_roll,0.35,'Roll', ...
    'HorizontalAlignment','center');

text(p_servo,0.35,'Servo', ...
    'HorizontalAlignment','center');

xlabel('Real Axis [1/sec]');
ylabel('Imaginary Axis [rad/sec]');
title('Open-Loop Poles and Zeros');
legend('Poles','Zeros','Location','best');