%% PARAMETERS --------------------------------------------------------------
m = 0.958;
b = 5.56e-5;
g = 9.81;

close all; clc;

%% Desired specs
tr = 12.5;
PO = 1;

omega = 2.2/tr;
zeta_d = -log(PO/100) / sqrt(pi^2 + (log(PO/100))^2);

ts1 = 4.6/(zeta_d*omega);
ts2 = ts1 * (3.9/4.6);

%% Controller gains
Kp = 6;
Ki = 2;
Kd = 4;

%% TRANSFER FUNCTIONS ------------------------------------------------------
s = tf('s');

G = 1/(m*s^2 + b*s);              % plant
C = Kp + Kd*s + Ki/s;             % PID
L = C*G;                          % open-loop
Tcl = feedback(L,1);              % closed-loop transfer function

%% POLES & ZEROS -----------------------------------------------------------

fprintf("\n=== Zeros and Poles of G(s) ===\n");
zG = zero(G)
pG = pole(G)

fprintf("\n=== Zeros and Poles of C(s) ===\n");
zC = zero(C)
pC = pole(C)

fprintf("\n=== Zeros and Poles of L(s)=C(s)G(s) ===\n");
zL = zero(L)
pL = pole(L)

fprintf("\n=== Zeros and Poles of T(s) Closed Loop ===\n");
zT = zero(Tcl)
pT = pole(Tcl)

%% Stability verdicts
fprintf("\nClosed-loop BIBO stability:\n");
if any(real(pT)>=0)
    fprintf("UNSTABLE or MARGINAL\n");
else
    fprintf("STABLE\n");
end

%% ------------------------------------------------------------------------
%% REQUIRED PLOTS
%% ------------------------------------------------------------------------

%% 1. Step Response
%figure;
%step(Tcl, 30);
%title("Closed-Loop Step Response");
%grid on;

%% 2. Disturbance Rejection (inject a step on plant input)
%DistTF = feedback(G, C);  % TF from disturbance to output
%figure;
%step(DistTF);
%title("Disturbance Response (gravity)");
%grid on;

%% 3. Bode Plots
%figure;
%bode(G); grid on; title("Bode Plot of G(s)");

%figure;
%bode(C); grid on; title("Bode Plot of C(s)");

figure;
bode(L); grid on; title("Bode Plot of Open-Loop L(s)=C(s)G(s)");

%% 4. Root Locus
%figure;
%rlocus(L);
%title("Root Locus of L(s)=C(s)G(s)");

%% 5. Simulink output plots -----------------------------------------------
if exist('out','var')
    t = out.h.Time;
    h = out.h.Data;
    F_r = out.Fr.Data;
    F_g = out.Fg.Data;
    F_net = out.Fnet.Data;

    figure;
    plot(t, h, 'LineWidth',1.5);
    xlabel("Time (s)"); ylabel("Height h(t)");
    title("h(t) above Delay Limit","FontSize", 18);
    grid on;

    %figure;
    %plot(t, F_r, 'LineWidth',1.5); hold on;
    %plot(t, F_g, 'LineWidth',1.5);
    %plot(t, F_net,'LineWidth',1.5);
    %legend("Reference Force F_r","Gravity F_g","Net Force");
    %title("Force Signals");
    %grid on;
end

%% ------------------------------------------------------------------------
%% Pole tables for report --------------------------------------------------
%% Plant G(s)
p = pG;
re = real(p); im = imag(p); mag = abs(p);
Z = -re./mag;                    % damping ratio
T_const = -1./re;                % time constant

Table_G = table(re,im,mag,Z,T_const, ...
    'VariableNames',{'Real','Imag','Abs','Zeta','Tau'})

%% Closed Loop
p = pT;
re = real(p); im = imag(p); mag = abs(p);
Z = -re./mag;
T_const = -1./re;

Table_T = table(re,im,mag,Z,T_const, ...
    'VariableNames',{'Real','Imag','Abs','Zeta','Tau'})



%% ------------------------------------------------------------------------
%% ROBUSTNESS ANALYSIS VALUES FOR LATEX REPORT
%% ------------------------------------------------------------------------

% Compute gain/phase margins and crossover frequencies
[Gm, Pm, Wcg, Wcp] = margin(L);

% Convert GM from absolute ratio to dB
Gm_dB = 20*log10(Gm);

% Compute allowable time delay at phase margin frequency
% T_delay_max = PM / wc  (PM must be in radians)
PM_rad = Pm * (pi/180);
T_delay_max = PM_rad / Wcp;

% Values just below / just above the delay margin for simulation tests
T_delay_below = 0.9 * T_delay_max;   % 10% below
T_delay_above = 1.1 * T_delay_max;   % 10% above

%% DISPLAY RESULTS ---------------------------------------------------------

fprintf("\n==================== ROBUSTNESS ANALYSIS ====================\n");

fprintf("Gain Margin (absolute):        %.4f\n", Gm);
fprintf("Gain Margin (dB):              %.4f dB\n", Gm_dB);
fprintf("Phase Margin:                  %.4f deg\n", Pm);
fprintf("Gain Crossover Frequency:      %.4f rad/s\n", Wcg);
fprintf("Phase Crossover Frequency:     %.4f rad/s\n", Wcp);

fprintf("\nMaximum Allowable Time Delay:  %.6f seconds\n", T_delay_max);
fprintf("Delay Just Below Margin:       %.6f seconds\n", T_delay_below);
fprintf("Delay Just Above Margin:       %.6f seconds\n", T_delay_above);

fprintf("==============================================================\n");