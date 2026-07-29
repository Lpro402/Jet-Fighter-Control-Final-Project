%% Q2(c) - Phase Lead final comparison
clear
clc
close all

s = tf('s');

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

%% Plant transfer functions

G_phi = minreal(tf(ss(A,B,C_phi,0)),1e-7);
G_da  = minreal(tf(ss(A,B,C_da,0)),1e-7);

%% Command

phi_cmd_deg = 30;
phi_cmd_rad = deg2rad(phi_cmd_deg);

t = (0:0.001:12)';
r = phi_cmd_rad*ones(size(t));

%% ============================================================
% 1. Lead trade-off designs
%% ============================================================

C_shape = (s+3)/(s+15);

K_lead_vec = [0.31044 0.39697 0.51562];
lead_labels = {'Lead 1 (\zeta\approx0.7)', ...
               'Lead 2 (\zeta\approx0.6)', ...
               'Lead 3 (\zeta\approx0.5)'};

phi_lead_all = zeros(length(t),length(K_lead_vec));

for i = 1:length(K_lead_vec)
    K_i = K_lead_vec(i);
    T_i = minreal(feedback(K_i*C_shape*G_phi,1),1e-7);
    phi_lead_all(:,i) = rad2deg(lsim(T_i,r,t));
end

figure
plot(t,phi_lead_all(:,1),'LineWidth',1.5)
hold on
plot(t,phi_lead_all(:,2),'LineWidth',1.5)
plot(t,phi_lead_all(:,3),'LineWidth',1.5)
yline(phi_cmd_deg,'k--','Command = 30 deg','LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('\phi [deg]')
title('Trade-Off Between Candidate Phase-Lead Designs')
legend(lead_labels{:},'Location','best')

%% ============================================================
% 2. Selected Phase Lead controller
%% ============================================================

K_lead = 0.39697;
C_lead = K_lead*C_shape;

T_lead_phi = minreal(feedback(C_lead*G_phi,1),1e-7);
T_lead_dc  = minreal(feedback(C_lead,G_phi),1e-7);
T_lead_da  = minreal(G_da*T_lead_dc,1e-7);

phi_lead = rad2deg(lsim(T_lead_phi,r,t));
da_lead  = rad2deg(lsim(T_lead_da,r,t));

final_lead = phi_cmd_deg*dcgain(T_lead_phi);
ess_lead = 100*(phi_cmd_deg-final_lead)/phi_cmd_deg;
info_lead = stepinfo(phi_lead,t,final_lead);
max_da_lead = max(abs(da_lead));

%% ============================================================
% 3. P controller from section 2(a)
%% ============================================================

K_P = 0.047953;

A_P = A-B*K_P*C_phi;
B_P = B*K_P;

sys_P_phi = ss(A_P,B_P,C_phi,0);
sys_P_da  = ss(A_P,B_P,C_da,0);

phi_P = rad2deg(lsim(sys_P_phi,r,t));
da_P  = rad2deg(lsim(sys_P_da,r,t));

final_P = phi_cmd_deg*dcgain(sys_P_phi);
ess_P = 100*(phi_cmd_deg-final_P)/phi_cmd_deg;
info_P = stepinfo(phi_P,t,final_P);
max_da_P = max(abs(da_P));

%% ============================================================
% 4. PD-like controller from section 2(b)
%% ============================================================

Kp_PD   = 0.01;
Kphi_PD = 0.07408;

A_PD = A-B*Kp_PD*C_p-B*Kphi_PD*C_phi;
B_PD = B*Kphi_PD;

sys_PD_phi = ss(A_PD,B_PD,C_phi,0);
sys_PD_da  = ss(A_PD,B_PD,C_da,0);

phi_PD = rad2deg(lsim(sys_PD_phi,r,t));
da_PD  = rad2deg(lsim(sys_PD_da,r,t));

final_PD = phi_cmd_deg*dcgain(sys_PD_phi);
ess_PD = 100*(phi_cmd_deg-final_PD)/phi_cmd_deg;
info_PD = stepinfo(phi_PD,t,final_PD);
max_da_PD = max(abs(da_PD));

%% ============================================================
% 5. Comparison plot - roll angle
%% ============================================================

figure
plot(t,phi_P,'LineWidth',1.5)
hold on
plot(t,phi_PD,'LineWidth',1.5)
plot(t,phi_lead,'LineWidth',1.8)
yline(phi_cmd_deg,'k--','Command = 30 deg','LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('\phi [deg]')
title('Roll-Angle Response Comparison')
legend('P','PD-like','Phase Lead','Location','best')

%% ============================================================
% 6. Comparison plot - aileron deflection
%% ============================================================

figure
plot(t,da_P,'LineWidth',1.5)
hold on
plot(t,da_PD,'LineWidth',1.5)
plot(t,da_lead,'LineWidth',1.8)
yline(5,'k--','+5 deg limit','LineWidth',1.2)
yline(-5,'k--','-5 deg limit','LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('\delta_a [deg]')
title('Aileron Deflection Comparison')
legend('P','PD-like','Phase Lead','Location','best')

%% ============================================================
% 7. Optional plot - tracking error comparison
%% ============================================================

e_P    = phi_cmd_deg - phi_P;
e_PD   = phi_cmd_deg - phi_PD;
e_lead = phi_cmd_deg - phi_lead;

figure
plot(t,e_P,'LineWidth',1.5)
hold on
plot(t,e_PD,'LineWidth',1.5)
plot(t,e_lead,'LineWidth',1.8)
grid on
xlabel('Time [s]')
ylabel('e(t) [deg]')
title('Tracking Error Comparison')
legend('P','PD-like','Phase Lead','Location','best')

%% ============================================================
% 8. Summary table
%% ============================================================

Controller = {'P'; 'PD-like'; 'Phase Lead'};
RiseTime = [info_P.RiseTime; info_PD.RiseTime; info_lead.RiseTime];
SettlingTime = [info_P.SettlingTime; info_PD.SettlingTime; info_lead.SettlingTime];
Overshoot = [info_P.Overshoot; info_PD.Overshoot; info_lead.Overshoot];
EssPercent = [ess_P; ess_PD; ess_lead];
MaxDeltaA = [max_da_P; max_da_PD; max_da_lead];

ResultsTable = table(Controller,RiseTime,SettlingTime,Overshoot,EssPercent,MaxDeltaA)

disp('Performance comparison table:')
disp(ResultsTable)