%% Question 3(a) - Systematic pole-placement search
% More than 4000 five-pole designs are evaluated.
%
% Selection rule:
% 1. Pole requirements must be satisfied.
% 2. Overshoot <= 5 percent.
% 3. Max aileron and command <= 5 deg.
% 4. Minimize settling time.
% 5. Break ties using rise time and then overshoot.

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

%% Controllability

Co = ctrb(A,B);

fprintf('Controllability rank = %d\n',rank(Co))

if rank(Co) ~= size(A,1)
    error('The system is not completely controllable.')
end

%% Design limits

zeta_min = 0.5;
zeta_max = 0.7;

wn_min = 2;
wn_max = 3;

real_min = 2;
real_max = 3;

overshoot_limit = 5;
aileron_limit = 5;
command_limit = 5;

%% Simulation settings for the search

phi_command_deg = 30;
phi_command_rad = deg2rad(phi_command_deg);

t_search = (0:0.005:8)';
r_search = phi_command_rad*ones(size(t_search));

%% ============================================================
% Stage 1 - Coarse search
%% ============================================================

zeta_values = 0.50:0.05:0.70;
wn_values   = 2.00:0.20:3.00;
real_values = 2.00:0.25:3.00;

[Z,W] = ndgrid(zeta_values,wn_values);

pair_list = [Z(:) W(:)];

coarse_parameters = [];

for i = 1:size(pair_list,1)-1

    for j = i+1:size(pair_list,1)

        for real_pole = real_values

            coarse_parameters(end+1,:) = [ ...
                pair_list(i,1), ...
                pair_list(i,2), ...
                pair_list(j,1), ...
                pair_list(j,2), ...
                real_pole]; %#ok<SAGROW>

        end
    end
end

fprintf('Coarse designs evaluated: %d\n', ...
    size(coarse_parameters,1))

CoarseResults = evaluateSet( ...
    coarse_parameters, ...
    A,B,C_phi, ...
    t_search,r_search,phi_command_deg);

coarse_valid = ...
    CoarseResults.Overshoot <= overshoot_limit & ...
    CoarseResults.MaxDeltaA <= aileron_limit & ...
    CoarseResults.MaxDeltaC <= command_limit;

CoarseValid = CoarseResults(coarse_valid,:);

CoarseValid = sortrows( ...
    CoarseValid, ...
    {'SettlingTime','RiseTime','Overshoot'}, ...
    {'ascend','ascend','ascend'});

best_coarse = CoarseValid(1,:);

%% ============================================================
% Stage 2 - Local refinement around the best coarse design
%% ============================================================

zeta_1_values = ...
    max(zeta_min,best_coarse.Zeta1-0.05):0.025: ...
    min(zeta_max,best_coarse.Zeta1+0.05);

zeta_2_values = ...
    max(zeta_min,best_coarse.Zeta2-0.05):0.025: ...
    min(zeta_max,best_coarse.Zeta2+0.05);

wn_1_values = ...
    max(wn_min,best_coarse.Wn1-0.20):0.05: ...
    min(wn_max,best_coarse.Wn1+0.20);

wn_2_values = ...
    max(wn_min,best_coarse.Wn2-0.20):0.05: ...
    min(wn_max,best_coarse.Wn2+0.20);

real_values_fine = ...
    max(real_min,best_coarse.RealMagnitude-0.20):0.05: ...
    min(real_max,best_coarse.RealMagnitude+0.20);

fine_parameters = [];

for zeta_1 = zeta_1_values

    for wn_1 = wn_1_values

        for zeta_2 = zeta_2_values

            for wn_2 = wn_2_values

                pair_1 = [zeta_1 wn_1];
                pair_2 = [zeta_2 wn_2];

                % Avoid repeated complex poles
                if norm(pair_1-pair_2) < 1e-10
                    continue
                end

                % Avoid equivalent designs with pair order reversed
                if pair_2(1) < pair_1(1) || ...
                   (abs(pair_2(1)-pair_1(1)) < 1e-10 && ...
                    pair_2(2) < pair_1(2))
                    continue
                end

                for real_pole = real_values_fine

                    fine_parameters(end+1,:) = [ ...
                        zeta_1,wn_1, ...
                        zeta_2,wn_2, ...
                        real_pole]; %#ok<SAGROW>

                end
            end
        end
    end
end

fprintf('Fine designs evaluated: %d\n', ...
    size(fine_parameters,1))

FineResults = evaluateSet( ...
    fine_parameters, ...
    A,B,C_phi, ...
    t_search,r_search,phi_command_deg);

fine_valid = ...
    FineResults.Overshoot <= overshoot_limit & ...
    FineResults.MaxDeltaA <= aileron_limit & ...
    FineResults.MaxDeltaC <= command_limit;

FineValid = FineResults(fine_valid,:);

FineValid = sortrows( ...
    FineValid, ...
    {'SettlingTime','RiseTime','Overshoot'}, ...
    {'ascend','ascend','ascend'});

%% Top 20 points

Top20 = FineValid(1:min(20,height(FineValid)),:);

Top20.Point = (1:height(Top20))';

Top20 = movevars(Top20,'Point','Before',1);

disp(' ')
disp('Top 20 valid pole-placement points:')

disp(Top20(:,{ ...
    'Point', ...
    'Zeta1','Wn1', ...
    'Zeta2','Wn2', ...
    'RealPole', ...
    'RiseTime', ...
    'SettlingTime', ...
    'Overshoot', ...
    'MaxDeltaA', ...
    'MaxDeltaC'}))

%% ============================================================
% Final selected point
%% ============================================================

SelectedPoint = Top20(1,:);

selected_parameters = [ ...
    SelectedPoint.Zeta1, ...
    SelectedPoint.Wn1, ...
    SelectedPoint.Zeta2, ...
    SelectedPoint.Wn2, ...
    SelectedPoint.RealMagnitude];

%% Accurate final simulation

t_final = (0:0.001:12)';
r_final = phi_command_rad*ones(size(t_final));

Final = evaluateOne( ...
    selected_parameters, ...
    A,B,C_phi, ...
    t_final,r_final,phi_command_deg);

fprintf('\n============================================\n')
fprintf('FINAL SELECTED DESIGN\n')
fprintf('============================================\n')

fprintf('zeta_1 = %.4f\n',selected_parameters(1))
fprintf('wn_1   = %.4f rad/s\n',selected_parameters(2))

fprintf('zeta_2 = %.4f\n',selected_parameters(3))
fprintf('wn_2   = %.4f rad/s\n',selected_parameters(4))

fprintf('Real pole = %.4f\n',-selected_parameters(5))

fprintf('\nDesired poles:\n')
disp(Final.DesiredPoles)

fprintf('State-feedback gain K:\n')
disp(Final.K)

fprintf('Nbar = %.8f\n',Final.Nbar)

fprintf('\nPerformance:\n')
fprintf('Rise time = %.4f s\n',Final.RiseTime)
fprintf('Settling time = %.4f s\n',Final.SettlingTime)
fprintf('Overshoot = %.4f %%\n',Final.Overshoot)
fprintf('Max |delta_a| = %.4f deg\n',Final.MaxDeltaA)
fprintf('Max |delta_c| = %.4f deg\n',Final.MaxDeltaC)
fprintf('Steady-state error = %.8f deg\n', ...
    Final.SteadyStateError)

%% ============================================================
% Comparison with the WORK design
%% ============================================================

work_parameters = [ ...
    0.65 2.20 ...
    0.60 2.80 ...
    2.50];

Work = evaluateOne( ...
    work_parameters, ...
    A,B,C_phi, ...
    t_final,r_final,phi_command_deg);

fprintf('\nWORK design comparison:\n')
fprintf('Rise time = %.4f s\n',Work.RiseTime)
fprintf('Settling time = %.4f s\n',Work.SettlingTime)
fprintf('Overshoot = %.4f %%\n',Work.Overshoot)
fprintf('Max |delta_a| = %.4f deg\n',Work.MaxDeltaA)
fprintf('Max |delta_c| = %.4f deg\n',Work.MaxDeltaC)

%% ============================================================
% Figure 1 - Five leading responses
%% ============================================================

figure('Name','Top Five Designs')
hold on
grid on

for i = 1:min(5,height(Top20))

    parameters_i = [ ...
        Top20.Zeta1(i), ...
        Top20.Wn1(i), ...
        Top20.Zeta2(i), ...
        Top20.Wn2(i), ...
        Top20.RealMagnitude(i)];

    Result_i = evaluateOne( ...
        parameters_i, ...
        A,B,C_phi, ...
        t_final,r_final,phi_command_deg);

    plot(t_final,Result_i.PhiDeg, ...
        'LineWidth',1.4, ...
        'DisplayName',sprintf('Point %d',i));
end

yline(phi_command_deg,'k--', ...
    'Command = 30 deg', ...
    'HandleVisibility','off');

xlabel('Time [s]')
ylabel('\phi [deg]')

title('Comparison of the Five Leading Pole Placements')

legend('Location','best')

xlim([0 6])

%% ============================================================
% Figure 2 - Final pole map with constraints
%% ============================================================

figure('Name','Final Pole Placement')
hold on
grid on
axis equal

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
    'DisplayName','Allowed complex-pole region');

fill(x_region,-y_region,[0.85 0.92 1.00], ...
    'FaceAlpha',0.5, ...
    'EdgeColor','none', ...
    'HandleVisibility','off');

wn_line = linspace(0,3.5,300);

for zeta_value = [zeta_min zeta_max]

    x_line = -zeta_value*wn_line;
    y_line = wn_line*sqrt(1-zeta_value^2);

    plot(x_line,y_line,'k--', ...
        'HandleVisibility','off');

    plot(x_line,-y_line,'k--', ...
        'HandleVisibility','off');
end

for wn_value = [wn_min wn_max]

    x_arc = -wn_value*cos(theta);
    y_arc = wn_value*sin(theta);

    plot(x_arc,y_arc,'k-', ...
        'HandleVisibility','off');

    plot(x_arc,-y_arc,'k-', ...
        'HandleVisibility','off');
end

plot([-real_max -real_min],[0 0], ...
    'LineWidth',7, ...
    'DisplayName','Allowed real-pole interval');

h_poles = scatter( ...
    real(Final.ActualPoles), ...
    imag(Final.ActualPoles), ...
    100, ...
    'x', ...
    'LineWidth',2.2, ...
    'DisplayName','Selected closed-loop poles');

wn_data = abs(Final.ActualPoles);
zeta_data = -real(Final.ActualPoles)./wn_data;

h_poles.DataTipTemplate.DataTipRows(1).Label = 'Real';
h_poles.DataTipTemplate.DataTipRows(2).Label = 'Imaginary';

h_poles.DataTipTemplate.DataTipRows(end+1) = ...
    dataTipTextRow('\omega_n',wn_data);

h_poles.DataTipTemplate.DataTipRows(end+1) = ...
    dataTipTextRow('\zeta',zeta_data);

xline(0,'k--','HandleVisibility','off')
yline(0,'k--','HandleVisibility','off')

xlabel('Real Axis [1/s]')
ylabel('Imaginary Axis [rad/s]')

title('Selected Closed-Loop Poles and Design Constraints')

legend('Location','best')

xlim([-4 0.3])
ylim([-3.4 3.4])

%% ============================================================
% Figure 3 - Final selected response
%% ============================================================

figure('Name','Final Selected Response')

plot(t_final,Final.PhiDeg,'LineWidth',1.8)
hold on

yline(phi_command_deg,'k--', ...
    'Command = 30 deg');

grid on

xlabel('Time [s]')
ylabel('\phi [deg]')

title('Roll-Angle Response of the Selected Design')

legend('\phi(t)','Command', ...
    'Location','best')

xlim([0 6])

%% ============================================================
% Figure 4 - Final control effort
%% ============================================================

figure('Name','Final Control Effort')

plot(t_final,Final.DeltaADeg,'LineWidth',1.8)
hold on

plot(t_final,Final.DeltaCDeg,'--','LineWidth',1.5)

yline(5,'k--','+5 deg limit', ...
    'HandleVisibility','off');

yline(-5,'k--','-5 deg limit', ...
    'HandleVisibility','off');

grid on

xlabel('Time [s]')
ylabel('Angle [deg]')

title('Aileron Deflection and Control Command')

legend('\delta_a(t)','\delta_c(t)', ...
    'Location','best')

xlim([0 6])

%% ============================================================
% Local functions
%% ============================================================

function Results = evaluateSet( ...
    Parameters,A,B,C_phi,t,r,phi_command_deg)

    number_of_designs = size(Parameters,1);

    RiseTime = zeros(number_of_designs,1);
    SettlingTime = zeros(number_of_designs,1);
    Overshoot = zeros(number_of_designs,1);

    MaxDeltaA = zeros(number_of_designs,1);
    MaxDeltaC = zeros(number_of_designs,1);

    SteadyStateError = zeros(number_of_designs,1);

    for i = 1:number_of_designs

        Result = evaluateOne( ...
            Parameters(i,:), ...
            A,B,C_phi,t,r,phi_command_deg);

        RiseTime(i) = Result.RiseTime;
        SettlingTime(i) = Result.SettlingTime;
        Overshoot(i) = Result.Overshoot;

        MaxDeltaA(i) = Result.MaxDeltaA;
        MaxDeltaC(i) = Result.MaxDeltaC;

        SteadyStateError(i) = ...
            Result.SteadyStateError;
    end

    Results = table( ...
        Parameters(:,1), ...
        Parameters(:,2), ...
        Parameters(:,3), ...
        Parameters(:,4), ...
       -Parameters(:,5), ...
        Parameters(:,5), ...
        RiseTime, ...
        SettlingTime, ...
        Overshoot, ...
        MaxDeltaA, ...
        MaxDeltaC, ...
        SteadyStateError, ...
        'VariableNames',{ ...
        'Zeta1', ...
        'Wn1', ...
        'Zeta2', ...
        'Wn2', ...
        'RealPole', ...
        'RealMagnitude', ...
        'RiseTime', ...
        'SettlingTime', ...
        'Overshoot', ...
        'MaxDeltaA', ...
        'MaxDeltaC', ...
        'SteadyStateError'});
end

function Result = evaluateOne( ...
    Parameters,A,B,C_phi,t,r,phi_command_deg)

    zeta_1 = Parameters(1);
    wn_1 = Parameters(2);

    zeta_2 = Parameters(3);
    wn_2 = Parameters(4);

    real_magnitude = Parameters(5);

    wd_1 = wn_1*sqrt(1-zeta_1^2);
    wd_2 = wn_2*sqrt(1-zeta_2^2);

    desired_poles = [ ...
        -zeta_1*wn_1 + 1i*wd_1;
        -zeta_1*wn_1 - 1i*wd_1;
        -zeta_2*wn_2 + 1i*wd_2;
        -zeta_2*wn_2 - 1i*wd_2;
        -real_magnitude];

    K = place(A,B,desired_poles);

    A_cl = A-B*K;

    Nbar = -1/(C_phi*(A_cl\B));

    sys_states = ss( ...
        A_cl, ...
        B*Nbar, ...
        eye(5), ...
        zeros(5,1));

    x = lsim(sys_states,r,t);

    phi_deg = rad2deg(x(:,4));
    delta_a_deg = rad2deg(x(:,5));

    delta_c_rad = -x*K.' + Nbar*r;
    delta_c_deg = rad2deg(delta_c_rad);

    sys_phi = ss(A_cl,B*Nbar,C_phi,0);

    phi_final_deg = ...
        phi_command_deg*dcgain(sys_phi);

    info = stepinfo( ...
        phi_deg, ...
        t, ...
        phi_final_deg, ...
        'SettlingTimeThreshold',0.02);

    Result.DesiredPoles = desired_poles;
    Result.ActualPoles = eig(A_cl);

    Result.K = K;
    Result.Nbar = Nbar;

    Result.PhiDeg = phi_deg;
    Result.DeltaADeg = delta_a_deg;
    Result.DeltaCDeg = delta_c_deg;

    Result.RiseTime = info.RiseTime;
    Result.SettlingTime = info.SettlingTime;
    Result.Overshoot = info.Overshoot;

    Result.MaxDeltaA = max(abs(delta_a_deg));
    Result.MaxDeltaC = max(abs(delta_c_deg));

    Result.SteadyStateError = ...
        phi_command_deg-phi_final_deg;
end