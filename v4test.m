clear; close; clc

% Initial state
X0 = zeros(12,1);
% X0(1) = 1;
% X0(2) = 1;
X0(3) = 10;     % heading rate, deg/s
X0(6) = 80;     % heading angle,  deg

% Control input: [pitch_ctl; phi_bat; VBD_ctlAD]
U = [0; 0; 0];

% Dummy aero coefficients (all zero -> no hydrodynamic forces/moments)
coefs.alphas = [-10 0 10];
coefs.betas  = [-10 0 10];
coefs.CLs    = zeros(3,3);
coefs.CDs    = zeros(3,3);
coefs.CYs    = zeros(3,3);
coefs.Croll  = zeros(3,3);
coefs.Cpitch = zeros(3,3);
coefs.Cyaw   = zeros(3,3);

% Glider geometry / inertias
params.S    = 1;
params.cbar = 1/4;
params.b    = 1;
params.heading_desired = 1;

params.Mf = zeros(3);
params.Jf = zeros(3);
params.Js = eye(3);
params.Ms   = 80;
params.mbat = 1;

% Random guess of static volume
params.Vstatic = 0.077;


% Ocean properties (new in V4)
params.rho      = 1025;   % seawater density, kg/m^3
params.ambtemp  = 15;     % ambient surface temp (deg C)
params.temp     = 8;     % local water temp    (deg C)
params.pressure = 101;      % pressure (surface)

% Run ODE
tic
[t, X] = ode45(@(t,X) dynamicsV4(X, U, coefs, params), [0 30], X0);
toc

% Heading rate
figure(1)
% plot(t,X(:,1),'b')   % roll rate
% plot(t,X(:,2),'b')   % pitch rate
plot(t,X(:,3),'b')
grid on
xlabel('Time (s)')
ylabel('Heading Rate (deg/s)')
title('Heading Rate at Unit Initial Velocity (dynamicsV4)')

% Heading angle
figure(2)
% plot(t,X(:,4),'b')   % roll
% plot(t,X(:,5),'b')   % pitch
plot(t,X(:,6),'b')
grid on
xlabel('Time (s)')
ylabel('Heading Angle (deg)')
title('Heading Angle at Zero Initial Velocity (dynamicsV4)')