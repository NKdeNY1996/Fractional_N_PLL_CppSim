clear 
close all
%%
Fs = 400e6;   %Sampling frequency
Ts = 1/Fs;
Nsamp_closed_loop = 10e6; 
Tstop = Nsamp_closed_loop*Ts;
%% Initial condition
IC1 = [0 0 0 0];%initial condition of each stage of MASH 
figure_index = 1;

%% Simulation Setup
% Selection of Ref Noise Source: 1:Ref injection, 2: CP injection

pfd_freq   = 20e6;       %reference frequency
Tpfd       = 1/pfd_freq;
ival       = 1.5e-3;
pre_gain   = 1/ival;
a0         = 0;          %PFD/CP polynomial nonlinearity
a1         = 1;
a2         = 0.01;
a3         = 0.00;
a4         = 0.0;
a5         = 0.01;
a6         = 0;
a7         = 0.0;
fzero      = 18.75e3;    %Loop Filter
fpole1     = 663e3;
fpole2     = 300e3;
fgain      = 229.14e6;
vco_kv     = 10e6;       %VCO gain
icp        = ival;       %Charge pump gain
alpha      = 1;          % 1: Tristate, 2: XOR-based
vco_fc     = 886.6e6;
Nint       = 45;         %Initger part of N
inp        = 1023;
M          = 1024^2;
Nnom       = Nint + inp/M; % Nominal division ratio

%% 
settle_sample  = 3e5;
icp_error      = 0;
%PFD Delay setup
pfd_reset_delay = 4*Ts;
pfd_down_xdelay = 0;%6*Ts;
% Sample and hold switch
s_and_h_en = 1; % activate sample and hold filter 1

% CP setup
icp_offset = 0;%-icp*((pfd_reset_delay+Ts)/Tpfd*e-pfd_down_xdelay/Tpfd*(1-e/2));%-1.2e-6

% Reference noise level
ref_noise = -500;%-150%
%%% CP injection
ref_noise_input        = -500;
icp_thermal_noise_up   = (alpha*icp)/(2*pi)*sqrt(10^(ref_noise/10)*2*50/4);
icp_thermal_noise_down = (alpha*icp)/(2*pi)*sqrt(10^(ref_noise/10)*2*50/4);
icp_flicker_corner_freq_up   = 1e3;
icp_flicker_corner_freq_down = 1e3;

% VCO setup
vco_foffset          = 1e6;
vco_noise_at_foffset = -145;%-125%-295%


%% Divider controller setup
name_list = {'MASH11'};
if strcmp(name_list,'MASH111')
    Divider_Controller = 2;    
elseif strcmp(name_list,'MASH11')
    Divider_Controller = 1;
else
    Divider_Controller = 0; % Integer
end

% LSB dither switch
LSB_Dither = 1;
%% Simulink Simulation
options         = simset('RelTol', 0, 'MaxStep', Ts);
% sim_closed_loop = sim('Simulink_poly_nonlinearity_pll_three_order_LFP_1.slx', Tstop, options); %CppSim model with ref noise source
sim_closed_loop = sim('Polynomial_model_for_locked_state_1.slx', Tstop, options); %CppSim model with ref noise source

% Get values

% load('./Data/Close_loop_MASH_111_linear');
noiseout = sim_closed_loop.noiseout;
sineout  = sim_closed_loop.sineout;
vin      = sim_closed_loop.vin;
icp_out  = sim_closed_loop.icp_out;

%% Calculation of Phase noise
% Sample from which loop is in steady state
% start_settle=round(100e-6/Ts); %100 us settling time, used in Rev1 and Rev2 figs
start_settle = 3e5; %200e3;%150 us settling time
noise = noiseout(start_settle:end)-mean(noiseout(start_settle:end));
% Phase noise calculation
Kv = 1; % VCO gain (Hz/V) - this is one for noiseout from CppSim
phase_noise = filter(2*pi*Kv/Fs,[1 -1],noise); %running sum of noise

%% Close-loop plot
font_size_1 = 16;
font_size_2 = 21;
side='twosided';
% Analysis window
RBW           = 1.5e2;
window_length = round(Fs/RBW);
window        = hann(window_length);
overlap       = round(0.5*window_length); %0.25 overlap was good
[Pxx,f_vec_closed_loop] = pwelch(phase_noise,window,overlap,window_length,Fs,side);

% Phase noise in dB scale
Pxx_db = 10*log10(Pxx);
figure(figure_index);figure_index = figure_index + 1;
semilogx(f_vec_closed_loop,Pxx_db);grid on;hold on;
ax = gca;
ax.XAxis.FontWeight = 'bold';  
ax.XAxis.FontSize = font_size_1;  
set(gcf, 'Position', [100, 100, 600, 450]);
ax.YAxis.FontSize = font_size_1; 
ax.YAxis.FontWeight = 'bold';
ax.YAxis.Color = 'k';
xlim([1e3,3e6]);
ylim([-145,-70]);
legend('Closed-loop','FontSize', font_size_2,'fontweight','bold','location','southwest');
xlabel('Frequency Offset(Hz)','FontSize', font_size_2,'fontweight','bold','Color', 'k');
ylabel('Phase Noise (dBc/Hz)','FontSize', font_size_2,'fontweight','bold','Color', 'k');
