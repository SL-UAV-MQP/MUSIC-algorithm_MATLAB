%% UAV Localization via Signals of Opportunity - MUSIC Algorithm
% Worcester Polytechnic Institute MQP Project
% 6-Element Uniform Circular Array for GNSS-Denied Navigation

clear; clc; close all;

%% ==================== PART 1: System Configuration ====================
% Operating frequency range: 700-1400 MHz
fc = 1050e6;              % Center frequency (Hz)
c = 3e8;                  % Speed of light (m/s)
lambda = c/fc;            % Wavelength (m)

% UCA Configuration
M = 6;                    % Number of antenna elements
radius = 0.6 * lambda;    % 0.176m as specified in report (0.6*lambda)
element_angles = (0:M-1) * 360/M; % 60 degrees apart

% Signal parameters
fs = 61.44e6;             % Sampling frequency (5G standard)
N = 8192;                 % Number of samples (increased for better estimation)
t = (0:N-1)/fs;          % Time vector

fprintf('=== MQP UAV Localization System ===\n');
fprintf('Center Frequency: %.1f MHz\n', fc/1e6);
fprintf('Array Radius: %.3f m (%.2f lambda)\n', radius, radius/lambda);
fprintf('Number of Antennas: %d\n', M);
fprintf('Wavelength: %.3f m\n\n', lambda);

%% ==================== PART 2: Simulate Signal Reception ====================
% Simulate 5G cell towers and P25 signals 
num_sources = 3;  % 3 signal sources (cell towers/P25 repeaters)
true_DOAs = [45, 135, 270];  % True directions of arrival (degrees)
source_powers = [1, 0.7, 0.5];  % Relative signal powers

fprintf('Simulating %d signal sources...\n', num_sources);
fprintf('True DOAs: %.1f°, %.1f°, %.1f°\n', true_DOAs);
fprintf('Source Powers: %.2f, %.2f, %.2f\n\n', source_powers);

% Generate received signals at UCA
X = zeros(M, N);
for src = 1:num_sources
    % Generate 5G-like OFDM signal
    signal = generate5GSignal(N, fs);
    signal = signal * source_powers(src);
    
    % Apply steering vector
    % Assuming signals arrive from horizontal plane (elevation = 90 deg)
    a = generateUCASteeringVector(true_DOAs(src), 90, M, radius, lambda);
    
    % Add signal contribution (proper matrix multiplication)
    X = X + a * signal;  % [M×1] * [1×N] = [M×N]
end

% Add noise (urban environment SNR)
SNR_dB = 15;  % Increased SNR for better comparison
X = awgn(X, SNR_dB, 'measured');

fprintf('SNR: %.1f dB\n\n', SNR_dB);

%% ==================== PART 3: Apply MUSIC Algorithm ====================
fprintf('Running MUSIC algorithm...\n');

% Define angle search grid
search_angles = 0:0.5:359.5;  % 0.5 degree resolution (720 points)

% Method 1: Standard MUSIC
[spectrum1, estimated_DOAs1, peaks1] = standardMUSIC(X, num_sources, search_angles, M, radius, lambda);

% Method 2: MUSIC with Forward-Backward Averaging (better for multipath)
[spectrum2, estimated_DOAs2, peaks2] = MUSICwithFBA(X, num_sources, search_angles, M, radius, lambda);

% Method 3: Spatial Smoothing MUSIC (added for comparison)
[spectrum3, estimated_DOAs3, peaks3] = MUSICwithSpatialSmoothing(X, num_sources, search_angles, M, radius, lambda);

%% ==================== PART 4: Visualize Results ====================
figure('Position', [100, 100, 1800, 600]);

% Plot 1: Standard MUSIC
subplot(1,3,1);
plot(search_angles, spectrum1, 'b-', 'LineWidth', 1.5);
hold on;
% Plot true DOAs
stem(true_DOAs, max(spectrum1)*ones(size(true_DOAs)), 'r^', ...
     'MarkerSize', 12, 'LineWidth', 2.5, 'MarkerFaceColor', 'r');
% Plot estimated DOAs
if ~isempty(estimated_DOAs1)
    stem(estimated_DOAs1, (max(spectrum1)-5)*ones(size(estimated_DOAs1)), ...
         'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
end
% Show peak heights
if ~isempty(peaks1)
    text_y = max(spectrum1) - 3;
    for i = 1:length(estimated_DOAs1)
        text(estimated_DOAs1(i), text_y, sprintf('%.1f dB', peaks1(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 9);
    end
end
xlabel('Azimuth Angle (degrees)', 'FontSize', 11);
ylabel('MUSIC Spectrum (dB)', 'FontSize', 11);
title('Standard MUSIC', 'FontSize', 13, 'FontWeight', 'bold');
legend('MUSIC Spectrum', 'True DOAs', 'Estimated DOAs', 'Location', 'best', 'FontSize', 9);
grid on;
xlim([0 360]);
ylim([min(spectrum1)-5 max(spectrum1)+5]);

% Plot 2: MUSIC with Forward-Backward Averaging
subplot(1,3,2);
plot(search_angles, spectrum2, 'b-', 'LineWidth', 1.5);
hold on;
% Plot true DOAs
stem(true_DOAs, max(spectrum2)*ones(size(true_DOAs)), 'r^', ...
     'MarkerSize', 12, 'LineWidth', 2.5, 'MarkerFaceColor', 'r');
% Plot estimated DOAs
if ~isempty(estimated_DOAs2)
    stem(estimated_DOAs2, (max(spectrum2)-5)*ones(size(estimated_DOAs2)), ...
         'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
end
% Show peak heights
if ~isempty(peaks2)
    text_y = max(spectrum2) - 3;
    for i = 1:length(estimated_DOAs2)
        text(estimated_DOAs2(i), text_y, sprintf('%.1f dB', peaks2(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 9);
    end
end
xlabel('Azimuth Angle (degrees)', 'FontSize', 11);
ylabel('MUSIC Spectrum (dB)', 'FontSize', 11);
title('MUSIC with FBA', 'FontSize', 13, 'FontWeight', 'bold');
legend('MUSIC Spectrum', 'True DOAs', 'Estimated DOAs', 'Location', 'best', 'FontSize', 9);
grid on;
xlim([0 360]);
ylim([min(spectrum2)-5 max(spectrum2)+5]);

% Plot 3: Spatial Smoothing MUSIC
subplot(1,3,3);
plot(search_angles, spectrum3, 'b-', 'LineWidth', 1.5);
hold on;
% Plot true DOAs
stem(true_DOAs, max(spectrum3)*ones(size(true_DOAs)), 'r^', ...
     'MarkerSize', 12, 'LineWidth', 2.5, 'MarkerFaceColor', 'r');
% Plot estimated DOAs
if ~isempty(estimated_DOAs3)
    stem(estimated_DOAs3, (max(spectrum3)-5)*ones(size(estimated_DOAs3)), ...
         'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
end
% Show peak heights
if ~isempty(peaks3)
    text_y = max(spectrum3) - 3;
    for i = 1:length(estimated_DOAs3)
        text(estimated_DOAs3(i), text_y, sprintf('%.1f dB', peaks3(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 9);
    end
end
xlabel('Azimuth Angle (degrees)', 'FontSize', 11);
ylabel('MUSIC Spectrum (dB)', 'FontSize', 11);
title('Spatial Smoothing MUSIC', 'FontSize', 13, 'FontWeight', 'bold');
legend('MUSIC Spectrum', 'True DOAs', 'Estimated DOAs', 'Location', 'best', 'FontSize', 9);
grid on;
xlim([0 360]);
ylim([min(spectrum3)-5 max(spectrum3)+5]);

%% ==================== PART 5: Display Results ====================
fprintf('\n=== MUSIC Algorithm Results ===\n');
fprintf('True DOAs:      [');
fprintf('%.1f° ', true_DOAs);
fprintf(']\n\n');

% Standard MUSIC results
fprintf('Standard MUSIC: [');
fprintf('%.1f° ', estimated_DOAs1);
fprintf(']\n');
fprintf('  Peak heights: [');
fprintf('%.1f ', peaks1);
fprintf('] dB\n');

% MUSIC with FBA results
fprintf('MUSIC with FBA: [');
fprintf('%.1f° ', estimated_DOAs2);
fprintf(']\n');
fprintf('  Peak heights: [');
fprintf('%.1f ', peaks2);
fprintf('] dB\n');

% Spatial Smoothing results
fprintf('Spatial Smooth: [');
fprintf('%.1f° ', estimated_DOAs3);
fprintf(']\n');
fprintf('  Peak heights: [');
fprintf('%.1f ', peaks3);
fprintf('] dB\n');

% Calculate and display errors
fprintf('\n=== Estimation Errors ===\n');

% Standard MUSIC
if length(estimated_DOAs1) == num_sources
    errors1 = calculateAngleErrors(estimated_DOAs1, true_DOAs);
    fprintf('Standard MUSIC: [');
    fprintf('%.2f° ', errors1);
    fprintf(']\n');
    fprintf('  Average Error: %.2f° | RMSE: %.2f°\n', mean(errors1), sqrt(mean(errors1.^2)));
end

% MUSIC with FBA
if length(estimated_DOAs2) == num_sources
    errors2 = calculateAngleErrors(estimated_DOAs2, true_DOAs);
    fprintf('MUSIC with FBA: [');
    fprintf('%.2f° ', errors2);
    fprintf(']\n');
    fprintf('  Average Error: %.2f° | RMSE: %.2f°\n', mean(errors2), sqrt(mean(errors2.^2)));
end

% Spatial Smoothing
if length(estimated_DOAs3) == num_sources
    errors3 = calculateAngleErrors(estimated_DOAs3, true_DOAs);
    fprintf('Spatial Smooth: [');
    fprintf('%.2f° ', errors3);
    fprintf(']\n');
    fprintf('  Average Error: %.2f° | RMSE: %.2f°\n', mean(errors3), sqrt(mean(errors3.^2)));
end

%% ==================== Additional Analysis ====================
fprintf('\n=== Performance Comparison ===\n');
fprintf('Dynamic Range:\n');
fprintf('  Standard MUSIC: %.1f dB\n', max(spectrum1) - median(spectrum1));
fprintf('  MUSIC with FBA: %.1f dB\n', max(spectrum2) - median(spectrum2));
fprintf('  Spatial Smooth: %.1f dB\n', max(spectrum3) - median(spectrum3));

%% ==================== HELPER FUNCTIONS ====================

function a = generateUCASteeringVector(theta, phi, M, radius, lambda)
    k = 2*pi/lambda;
    theta_rad = deg2rad(theta);
    phi_rad = deg2rad(phi);
    element_angles = (0:M-1) * 2*pi/M;
    
    a = zeros(M, 1);
    for m = 1:M
        phase = k * radius * sin(phi_rad) * cos(theta_rad - element_angles(m));
        a(m) = exp(1j * phase);
    end
    a = a / sqrt(M);
end

function signal = generate5GSignal(N, fs)
    signal = zeros(1, N);
    ssb_period = floor(fs * 0.02);
    
    pss = zadoffChuSeq(127, 29);
    pss_len = length(pss);
    sss = mSeq(127);
    sss_len = length(sss);
    pbch_len = 240;
    pbch = (2*randi([0,1], 1, pbch_len) - 1) + 1j*(2*randi([0,1], 1, pbch_len) - 1);
    pbch = pbch / sqrt(2);
    
    num_blocks = floor(N / ssb_period);
    for k = 0:num_blocks-1
        start_idx = k * ssb_period + 1;
        if start_idx + pss_len - 1 <= N
            signal(start_idx:start_idx+pss_len-1) = pss;
        end
        sss_start = start_idx + pss_len + 10;
        if sss_start + sss_len - 1 <= N
            signal(sss_start:sss_start+sss_len-1) = sss;
        end
        pbch_start = sss_start + sss_len + 10;
        if pbch_start + pbch_len - 1 <= N
            signal(pbch_start:pbch_start+pbch_len-1) = pbch;
        end
    end
    
    empty_slots = (signal == 0);
    num_empty = sum(empty_slots);
    data_symbols = (2*randi([0,1], 1, num_empty) - 1) + 1j*(2*randi([0,1], 1, num_empty) - 1);
    data_symbols = data_symbols / sqrt(2) * 0.3;
    signal(empty_slots) = data_symbols;
end

function [spectrum, estimated_DOAs, peak_heights] = standardMUSIC(X, num_signals, search_angles, M, radius, lambda)
    [~, N_samples] = size(X);
    Rxx = (X * X') / N_samples;
    
    [V, D] = eig(Rxx);
    [~, idx] = sort(diag(D), 'descend');
    V = V(:, idx);
    Un = V(:, num_signals+1:end);
    
    spectrum = zeros(length(search_angles), 1);
    for k = 1:length(search_angles)
        a = generateUCASteeringVector(search_angles(k), 90, M, radius, lambda);
        spectrum(k) = 1 / abs(a' * Un * Un' * a);
    end
    
    spectrum = 10*log10(abs(spectrum));
    spectrum = spectrum - max(spectrum);
    
    noise_floor = median(spectrum);
    threshold = noise_floor + 20;
    threshold = max(threshold, -15);
    
    [pks, locs] = findpeaks(spectrum, ...
                           'MinPeakHeight', threshold, ...
                           'NPeaks', num_signals*2, ...
                           'SortStr', 'descend', ...
                           'MinPeakDistance', 10);
    
    if length(pks) > num_signals
        pks = pks(1:num_signals);
        locs = locs(1:num_signals);
    end
    
    estimated_DOAs = search_angles(locs);
    peak_heights = pks;
    [estimated_DOAs, sort_idx] = sort(estimated_DOAs);
    peak_heights = peak_heights(sort_idx);
end

function [spectrum, estimated_DOAs, peak_heights] = MUSICwithFBA(X, num_signals, search_angles, M, radius, lambda)
    % FBA with better peak selection
    
    [M_ant, N_samples] = size(X);
    J = fliplr(eye(M_ant));
    
    Rf = (X * X') / N_samples;
    Rb = J * conj(Rf) * J;
    Rxx = (Rf + Rb) / 2;
    
    [V, D] = eig(Rxx);
    [~, idx] = sort(diag(D), 'descend');
    V = V(:, idx);
    Un = V(:, num_signals+1:end);
    
    spectrum = zeros(length(search_angles), 1);
    for k = 1:length(search_angles)
        a = generateUCASteeringVector(search_angles(k), 90, M_ant, radius, lambda);
        spectrum(k) = 1 / abs(a' * Un * Un' * a);
    end
    
    spectrum = 10*log10(abs(spectrum));
    spectrum = spectrum - max(spectrum);
    
    % More aggressive peak selection
    % Find ALL significant peaks first
    noise_floor = median(spectrum);
    threshold = noise_floor + 18;  % Slightly lower threshold
    threshold = max(threshold, -18);
    
    [all_pks, all_locs] = findpeaks(spectrum, ...
                                    'MinPeakHeight', threshold, ...
                                    'SortStr', 'descend', ...
                                    'MinPeakDistance', 8);  % Smaller distance
    
    % Select peaks more intelligently
    if length(all_pks) > num_signals
        % Method 1: Check for aliases (angles near 0/360 boundary)
        selected_locs = [];
        selected_pks = [];
        
        for i = 1:length(all_locs)
            angle = search_angles(all_locs(i));
            
            % Check if this is an alias of an already selected peak
            is_alias = false;
            for j = 1:length(selected_locs)
                selected_angle = search_angles(selected_locs(j));
                angle_diff = min(abs(angle - selected_angle), 360 - abs(angle - selected_angle));
                
                % If within 30 degrees, consider it as potential alias
                if angle_diff < 30
                    % Keep the one with higher peak
                    if all_pks(i) > selected_pks(j)
                        selected_locs(j) = all_locs(i);
                        selected_pks(j) = all_pks(i);
                    end
                    is_alias = true;
                    break;
                end
            end
            
            if ~is_alias && length(selected_locs) < num_signals
                selected_locs = [selected_locs, all_locs(i)];
                selected_pks = [selected_pks, all_pks(i)];
            end
            
            if length(selected_locs) >= num_signals
                break;
            end
        end
        
        locs = selected_locs;
        pks = selected_pks;
    else
        locs = all_locs;
        pks = all_pks;
    end
    
    estimated_DOAs = search_angles(locs);
    peak_heights = pks;
    [estimated_DOAs, sort_idx] = sort(estimated_DOAs);
    peak_heights = peak_heights(sort_idx);
end

function [spectrum, estimated_DOAs, peak_heights] = MUSICwithSpatialSmoothing(X, num_signals, search_angles, M, radius, lambda)
    % MUSIC with Spatial Smoothing
    % Reduces effects of coherent signals and improves resolution
    
    [M_ant, N_samples] = size(X);
    
    % Spatial smoothing parameters
    L = 4;  % Subarray size (M should be >= 2L-1, we have M=6, so L=4 is max)
    
    % Forward smoothing
    Rxx_smooth = zeros(L, L);
    num_subarrays = M_ant - L + 1;
    
    for i = 1:num_subarrays
        X_sub = X(i:i+L-1, :);
        Rxx_smooth = Rxx_smooth + (X_sub * X_sub') / N_samples;
    end
    Rxx_smooth = Rxx_smooth / num_subarrays;
    
    % Eigenvalue decomposition
    [V, D] = eig(Rxx_smooth);
    [~, idx] = sort(diag(D), 'descend');
    V = V(:, idx);
    Un = V(:, num_signals+1:end);
    
    % MUSIC spectrum (using reduced array)
    spectrum = zeros(length(search_angles), 1);
    for k = 1:length(search_angles)
        % Generate steering vector for subarray
        a_full = generateUCASteeringVector(search_angles(k), 90, M_ant, radius, lambda);
        a = a_full(1:L);  % Take first L elements
        a = a / norm(a);  % Renormalize
        
        spectrum(k) = 1 / abs(a' * Un * Un' * a);
    end
    
    spectrum = 10*log10(abs(spectrum));
    spectrum = spectrum - max(spectrum);
    
    % Peak detection
    noise_floor = median(spectrum);
    threshold = noise_floor + 20;
    threshold = max(threshold, -15);
    
    [pks, locs] = findpeaks(spectrum, ...
                           'MinPeakHeight', threshold, ...
                           'NPeaks', num_signals*2, ...
                           'SortStr', 'descend', ...
                           'MinPeakDistance', 10);
    
    if length(pks) > num_signals
        pks = pks(1:num_signals);
        locs = locs(1:num_signals);
    end
    
    estimated_DOAs = search_angles(locs);
    peak_heights = pks;
    [estimated_DOAs, sort_idx] = sort(estimated_DOAs);
    peak_heights = peak_heights(sort_idx);
end

function errors = calculateAngleErrors(estimated, true_angles)
    errors = zeros(size(estimated));
    for i = 1:length(estimated)
        ang_diffs = abs(estimated(i) - true_angles);
        ang_diffs = min(ang_diffs, 360 - ang_diffs);
        [min_error, ~] = min(ang_diffs);
        errors(i) = min_error;
    end
end

function seq = zadoffChuSeq(N, root)
    n = 0:N-1;
    seq = exp(-1j * pi * root * n .* (n + 1) / N);
end

function seq = mSeq(N)
    seq = (2*randi([0,1], 1, N) - 1) + 1j*(2*randi([0,1], 1, N) - 1);
    seq = seq / sqrt(2);
end