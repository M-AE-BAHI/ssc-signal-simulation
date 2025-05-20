%{
This MATLAB script simulates the Spread Spectrum Clock (SSC) spectrum, modulated
using sawtooth profile function. The simulation aims to analyze and visualize the spectral
properties of the SSC-modulation technique.
For a detailed explanation of the underlying equations and theory, please refer to:
Mohamed Alla Eddine Bahi, Maria Mendez Real, Erwan Nogues, Maxime Pelcat,
"Clock-to-Clock Modulation Covert Channel", Accepted for presentation at EMC Europe, 
Paris, France, 2025.
HAL Archive: ⟨hal-05069600⟩
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define parameters for signal generation
f0 = 100e6;  % Carrier frequency of the square wave (100 MHz)
Fs0 = 100 * f0; % Sampling frequency for square wave (100x f0 for aliasing avoidance)
T0 = 1 / f0; % Period of the square wave
cycle = 10^5; % Number of cycles for simulation
t0 = 0:1/Fs0:cycle*T0; % Time vector with sufficient resolution for square wave

% Generate square wave
x0 = square(2 * pi * f0 * t0); 

% Define parameters for sawtooth modulation
f1 = 30e3;   % Frequency of sawtooth modulating signal (30 kHz)
x1 = sawtooth(2 * pi * f1 * t0, 0.5); % This is in the case of center-spreading
% in the case of up-spreading :  0.5*sawtooth(2 * pi * f1 * t0, 0.5) + 0.5;
% in the case of down-spreading 0.5*sawtooth(2 * pi * f1 * t0, 0.5)-0.5;

% Modulation depth and modulated frequency
delta = 0.01; % Modulation depth (1% frequency spread for SSC)
f_modulated = f0 * (1 + delta * x1); % Frequency modulation using sawtooth wave

% Generate SSC modulated square wave using numerical integration (cumtrapz)
phase = cumtrapz(t0, 2 * pi * f_modulated); % Numerical integration of phase
x2 = square(phase); % SSC modulated square wave signal

% FFT Analysis
N = length(t0); % Number of points in FFT
X0 = fftshift(abs(fft(x0)) / N); % FFT and normalization of original square wave
X2 = fftshift(abs(fft(x2)) / N); % FFT and normalization of SSC modulated square wave

% Convert FFT magnitude to decibels (dB)
X0_dB = 20 * log10(X0 + eps); % Convert to dB scale (eps avoids log(0))
X2_dB = 20 * log10(X2 + eps); % Convert to dB scale

% Frequency vector for FFT plots
freq = (-N/2:N/2-1) * (Fs0 / N); % Generate frequency axis in Hz

% Plot results
figure;

% Plot the original square wave in the time domain
subplot(2, 2, 1);
plot(t0 * 1e6, x0); % Convert time to microseconds for display
xlabel('Time [\mu s]');
ylabel('Amplitude');
title('Time Domain: Square Wave (100 MHz)');
xlim([0 0.1]); % Zoom to show first 0.1 ?s
grid on;

% Plot the SSC modulated square wave in the time domain
subplot(2, 2, 2);

plot(t0 * 1e6, x2, 'r', 'LineWidth', 1.5); % Modulated square wave in red
hold on;
plot(t0 * 1e6, x0, 'b', 'LineWidth', 1.5); % Original square wave in blue
xlabel('Time [\mu s]');
ylabel('Amplitude');
title('Time Domain: SS Modulated vs Non-modulated');
legend('SS Modulated', 'Non-modulated', 'Location', 'best','bold');
xlim([0 0.2]);
ylim([-1.5 1.5]);
grid on;
hold on;

% Plot the FFT of the original square wave in the frequency domain
subplot(2, 2, 3);
plot(freq * 1e-6, X0_dB, 'b'); % Convert frequency to MHz for display
xlabel('Frequency [MHz]');
ylabel('Magnitude [dB]');
title('Frequency Domain: Square Wave (100 MHz)');
xlim([90 110]); % Focus on the vicinity of the carrier frequency
grid on;

% Plot the FFT of the SSC modulated square wave in the frequency domain
subplot(2, 2, 4); 

plot(freq * 1e-6, X2_dB, 'r','LineWidth', 1.5); % SSC modulated spectrum in red
hold on;
plot(freq * 1e-6, X0_dB, 'b','LineWidth', 1.5); % Original spectrum in blue
xlabel('Frequency [MHz]');
ylabel('Magnitude [dB]');
title('Frequency Domain: SS Modulated vs Non-modulated');
legend('SS Modulated', 'Non-modulated', 'Location', 'best','bold');
xlim([98 102]); % Focus on the vicinity of the carrier frequency
grid on;
hold off;
