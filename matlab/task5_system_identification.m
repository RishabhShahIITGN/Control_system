%% Task 5: System Characterization and Identification
% Ball-and-Beam / Sliding Object System
%
% This script implements Task 5 of the Control Systems Laboratory:
% 1. Loads chirp actuator and ultrasonic distance sensor data from CSV.
% 2. Filters out corrupted runs (drops amp30 with -1 sensor dropouts).
% 3. Resamples data to a uniform 50 Hz grid (Ts = 0.02 s) and detrends.
% 4. Estimates a continuous-time transfer function using n4sid (subspace method).
% 5. Analyzes the Bode plot, frequency response, gain/phase behavior, and stability margins.
%
% Author: Control Systems Laboratory
% Date: November 2026

clear; close all; clc;

%% 1. Configuration and Paths
candidatePaths = {
    fullfile('..', 'data', 'task5_chirp_runs.csv'), ... % Running from matlab/ folder
    fullfile('data', 'task5_chirp_runs.csv'),        ... % Running from repo root
    'task5_chirp_runs.csv'                           ... % Placed in same folder (e.g. MATLAB Online)
};

dataPath = '';
for k = 1:length(candidatePaths)
    if exist(candidatePaths{k}, 'file') == 2
        dataPath = candidatePaths{k};
        break;
    end
end

assert(~isempty(dataPath), ['Data file task5_chirp_runs.csv not found! ', ...
    'Please ensure task5_chirp_runs.csv is uploaded in the same folder or in a data/ subfolder.']);

% Set output directory for saving figures
outputDir = fileparts(mfilename('fullpath'));
if isempty(outputDir)
    outputDir = pwd;
end

fprintf('=======================================================\n');
fprintf('  TASK 5: SYSTEM CHARACTERIZATION VIA CHIRP EXCITATION \n');
fprintf('=======================================================\n');

%% 2. Load and Preprocess Data
opts = detectImportOptions(dataPath);
opts.VariableNamingRule = 'preserve';
dataTable = readtable(dataPath, opts);

% Target sampling specifications
Ts = 0.02;          % 50 Hz target sampling period (s)
fs = 1 / Ts;        % 50 Hz
t_grid = (0.1:Ts:19.9)'; % Uniform time grid for 20 s chirp

% Select clean runs (amp20 and amp25 runs; amp30 excluded due to ultrasonic dropouts)
clean_run_ids = {'amp20_run1', 'amp25_run1', 'amp20_run2', 'amp25_run2'};
num_runs = length(clean_run_ids);

id_datasets = cell(num_runs, 1);
time_data = cell(num_runs, 1);

figure('Name', 'Task 5: Preprocessed Actuator Chirp Inputs and Sensor Outputs', ...
       'Color', 'w', 'Position', [100, 100, 1000, 700]);

for i = 1:num_runs
    rid = clean_run_ids{i};
    mask = strcmp(dataTable.run_id, rid);
    
    t_raw = dataTable.time_ms(mask) / 1000.0; % Convert ms to seconds
    u_raw = dataTable.servo_deg(mask);
    y_raw = dataTable.distance_cm(mask);
    
    % Interpolate onto uniform 50 Hz time grid
    u_res = interp1(t_raw, u_raw, t_grid, 'linear', 'extrap');
    y_res = interp1(t_raw, y_raw, t_grid, 'linear', 'extrap');
    
    % Moving Average Offset Removal:
    % 1. Actuator Input: Center around operating point
    u_detrend = detrend(u_res, 'constant');
    
    % 2. Sensor Output: Subtract moving average baseline (window = 2.0s = 100 samples)
    % This removes the non-linear downhill drift/offset so the signal oscillates symmetrically about 0.
    movWindow = round(2.0 / Ts); % 100 samples at 50 Hz
    y_baseline = movmean(y_res, movWindow);
    y_detrend  = y_res - y_baseline;
    
    % Store iddata object for MATLAB System Identification Toolbox
    id_datasets{i} = iddata(y_detrend, u_detrend, Ts, 'Name', rid);
    id_datasets{i}.InputName = 'Servo Command';
    id_datasets{i}.InputUnit = 'deg';
    id_datasets{i}.OutputName = 'Sensor Distance';
    id_datasets{i}.OutputUnit = 'cm';
    
    time_data{i}.t = t_grid;
    time_data{i}.u = u_res;
    time_data{i}.y = y_res;
    time_data{i}.u_detrend = u_detrend;
    time_data{i}.y_detrend = y_detrend;
    
    % Plot zero-centered runs
    subplot(num_runs, 2, 2*i - 1);
    plot(t_grid, u_detrend, 'LineWidth', 1.2, 'Color', [0 0.4470 0.7410]);
    yline(0, '--', 'Color', [0.5 0.5 0.5]);
    grid on;
    ylabel(sprintf('%s\n\\Delta\\theta (deg)', rid), 'FontWeight', 'bold');
    if i == 1, title('Centered Actuator Input \Delta u(t)', 'FontSize', 11); end
    if i == num_runs, xlabel('Time (s)'); end
    
    subplot(num_runs, 2, 2*i);
    plot(t_grid, y_detrend, 'LineWidth', 1.2, 'Color', [0.85 0.33 0.1]);
    yline(0, '--', 'Color', [0.5 0.5 0.5]);
    grid on;
    ylabel('\Delta d (cm)', 'FontWeight', 'bold');
    if i == 1, title('Moving-Average Centered Distance \Delta y(t)', 'FontSize', 11); end
    if i == num_runs, xlabel('Time (s)'); end
end

saveas(gcf, fullfile(outputDir, 'task5_chirp_runs.png'));

%% 3. Merge Datasets for Multi-Experiment Subspace Identification
% Combine all clean experiments into a multi-experiment iddata object
merged_data = merge(id_datasets{1}, id_datasets{2}, id_datasets{3}, id_datasets{4});

%% 4. Subspace State-Space Identification (n4sid)
% The ball-and-beam/sliding object dynamics physically form a 2nd-order system
% (position and velocity states), potentially with actuator/sensor lag.
% Order nx = 2 captures dominant mechanical dynamics.
nx = 2;
opt = n4sidOptions('Focus', 'simulation', 'Display', 'on');
sys_d = n4sid(merged_data, nx, opt);

% Convert discrete-time model to continuous-time
try
    sys_c = d2c(sys_d, 'zoh');
catch
    sys_c = d2c(sys_d, 'tustin');
end

% Extract Continuous-time Transfer Function G(s)
[num, den] = tfdata(sys_c, 'v');
G = tf(num, den);
G.InputName = 'Servo Angle (\Delta\theta)';
G.InputUnit = 'deg';
G.OutputName = 'Distance (\Delta d)';
G.OutputUnit = 'cm';

fprintf('\n-------------------------------------------------------\n');
fprintf('  IDENTIFIED CONTINUOUS TRANSFER FUNCTION G(s)        \n');
fprintf('-------------------------------------------------------\n');
G

% Poles, Zeros, Damping and Natural Frequency
poles = pole(G);
zeros = zero(G);
[wn, zeta] = damp(G);

fprintf('Poles:\n');
for p = 1:length(poles)
    fprintf('  p%d = %+.4f %+.4fj  (wn = %.4f rad/s, zeta = %.4f)\n', ...
            p, real(poles(p)), imag(poles(p)), wn(p), zeta(p));
end

if isempty(zeros)
    fprintf('Zeros: None\n');
else
    fprintf('Zeros:\n');
    for z = 1:length(zeros)
        fprintf('  z%d = %+.4f %+.4fj\n', z, real(zeros(z)), imag(zeros(z)));
    end
end

DC_gain = dcgain(G);
fprintf('DC Gain: %.4f cm/deg (%.2f dB)\n', DC_gain, 20*log10(abs(DC_gain)));

%% 5. Model Validation and Fit Comparison
figure('Name', 'Task 5: Model Fit Comparison (n4sid vs Measured Data)', ...
       'Color', 'w', 'Position', [150, 150, 1000, 600]);
compare(merged_data, sys_d);
grid on;
title('Model Validation: Measured Output vs. n4sid Simulated Response', 'FontSize', 12, 'FontWeight', 'bold');
saveas(gcf, fullfile(outputDir, 'task5_model_validation.png'));

%% 6. Bode Plot and Stability Margin Analysis
figure('Name', 'Task 5: Bode Diagram and Stability Margins', ...
       'Color', 'w', 'Position', [200, 200, 900, 650]);
margin(G);
grid on;
title('Bode Diagram of Identified Plant G(s) = \Delta d(s) / \Delta\theta(s)', ...
      'FontSize', 13, 'FontWeight', 'bold');

[Gm, Pm, Wcg, Wcp] = margin(G);
fprintf('\n-------------------------------------------------------\n');
fprintf('  FREQUENCY RESPONSE AND STABILITY MARGINS             \n');
fprintf('-------------------------------------------------------\n');
if isinf(Gm)
    fprintf('Gain Margin (Gm):           Inf dB\n');
else
    fprintf('Gain Margin (Gm):           %.2f dB (at %.4f rad/s = %.4f Hz)\n', ...
            20*log10(Gm), Wcg, Wcg/(2*pi));
end
if isinf(Pm)
    fprintf('Phase Margin (Pm):          Inf deg\n');
else
    fprintf('Phase Margin (Pm):          %.2f deg (at %.4f rad/s = %.4f Hz)\n', ...
            Pm, Wcp, Wcp/(2*pi));
end
fprintf('Gain Crossover Freq (Wcp):  %.4f rad/s (%.4f Hz)\n', Wcp, Wcp/(2*pi));
fprintf('Phase Crossover Freq (Wcg): %.4f rad/s (%.4f Hz)\n', Wcg, Wcg/(2*pi));

saveas(gcf, fullfile(outputDir, 'task5_bode_plot.png'));

%% 7. Pole-Zero Map
figure('Name', 'Task 5: Pole-Zero Map', 'Color', 'w', 'Position', [250, 250, 600, 500]);
pzmap(G);
grid on;
title('Pole-Zero Map of Identified System G(s)', 'FontSize', 12, 'FontWeight', 'bold');
saveas(gcf, fullfile(outputDir, 'task5_pzmap.png'));

%% 8. Step Response Comparison: Identified vs. Ideal Sliding Car Model
% Ideal physics for sliding car on a beam: double integrator G_ideal(s) = K_ideal / s^2
% Dynamics: m * x_ddot = m * g * sin(alpha) => x_ddot = g * alpha = g * (d/L) * theta
% (No rotational inertia factor because the object is a sliding car)
% For L = 0.40 m, d = 0.04 m, g = 9.81 m/s^2:
% K_ideal = g * (d/L) * (100 * pi / 180) ~ 1.7121 cm/s^2 per deg
L_beam  = 0.40;   % Beam length (m)
d_arm   = 0.04;   % Lever arm length (m)
g_acc   = 9.81;   % Gravity (m/s^2)
deg2rad = pi / 180;
m2cm    = 100;

% Ideal sliding car gain (cm/s^2 per deg):
K_ideal = g_acc * (d_arm / L_beam) * (m2cm * deg2rad); % ~1.7121 cm/s^2/deg
G_ideal = tf(K_ideal, [1, 0, 0]);

% Time vector for step evaluation (0 to 3 seconds)
t_step = 0:0.01:3.0;

% Step response for a 1-degree servo step input (\Delta\theta = 1 deg)
[y_ident, t_out] = step(G, t_step);

% Match physical direction: if sensor distance decreases when servo tilts up
sign_dir = sign(dcgain(G));
if sign_dir == 0, sign_dir = 1; end
y_ideal_dir = sign_dir * 0.5 * K_ideal * (t_step.^2);

figure('Name', 'Task 5: Step Response Comparison', 'Color', 'w', 'Position', [300, 200, 900, 700]);

% Subplot 1: Zoomed in to millimeters to clearly see the identified model's transient dynamics
subplot(2, 1, 1);
plot(t_out, y_ident * 10, 'b-', 'LineWidth', 2.2);
grid on;
ylabel('\Delta d (mm)', 'FontWeight', 'bold', 'FontSize', 11);
title('Identified Plant Step Response (Zoomed to Millimeter Scale to View Transient Dynamics)', ...
      'FontSize', 11, 'FontWeight', 'bold');
legend(sprintf('Identified G(s) (Settles at %.3f mm)', y_ident(end)*10), 'Location', 'best');

% Subplot 2: Comparison with the ideal frictionless runaway double-integrator
subplot(2, 1, 2);
plot(t_out, y_ident, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Identified Model G(s) (Stabilized)');
hold on;
plot(t_step, y_ideal_dir, 'r--', 'LineWidth', 2.0, ...
     'DisplayName', sprintf('Ideal Sliding Car (Runaway: %+.3f / s^2)', sign_dir * K_ideal));
grid on;
xlabel('Time (seconds)', 'FontSize', 11);
ylabel('Position Deviation \Delta d (cm)', 'FontSize', 11, 'FontWeight', 'bold');
title('Step Response Comparison: Stabilized Hardware Model vs. Ideal Frictionless Runaway', ...
      'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
xlim([0, 3.0]);

saveas(gcf, fullfile(outputDir, 'task5_step_response_comparison.png'));

fprintf('\n-------------------------------------------------------\n');
fprintf('  STEP RESPONSE & SLIDING CAR MODEL VERIFICATION       \n');
fprintf('-------------------------------------------------------\n');
fprintf('Ideal Sliding Car Gain (K_ideal):    %.4f cm/s^2 per deg\n', K_ideal);
fprintf('Identified Model Numerator (K_ident): %.4f\n', num(end));
fprintf('Step response comparison saved to task5_step_response_comparison.png\n');

fprintf('\nSystem Identification and Characterization Complete.\n');
fprintf('Figures saved to matlab/ folder.\n');
