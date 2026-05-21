% Seaglider dynamics function for digital twin via momentum
% To be put through your favorite ODE solver
% Author: Kyle Wittcoff
% Date: 14 May 2026
% Send all complaints to: president@whitehouse.gov
% May Poseidon help us...

function X_dyn = dynamicsV6(X,U,coefs,params)
%% Unpacking Inputs

% Avoiding divide by zero errors

% Unpacking state vector X
phidot = X(1) * pi/180; % roll rate (deg/s to rad/s)
thetadot = X(2) * pi/180; % pitch rate (deg/s to rad/s)
headingdot = X(3) * pi/180; % yaw rate (deg/s to rad/s)

phi = X(4) * pi/180; % roll angle (deg to rad)
theta = X(5) * pi/180; % pitch angle (deg to rad)
heading = X(6) * pi/180; % heading angle (deg to rad)

u = X(7); % horizontal velocity in glide path frame
v = X(8); % lateral velocity in glide path frame
w = X(9); % vertical velocity in glide path frame

x = X(10); % x distance in Glide Path frame
y = X(11); % y distance in Glide Path frame
z = X(12); % z distance (depth) in Glide Path frame

% Unpacking control input
pitch_ctl = U(1); % battery pos down x axis (in body frame, aft positive)
phi_bat = U(2); % battery roll angle
VBD_cc = U(3); % oil count in the VBD pump in AD

x_bat = - pitch_ctl;

% Unpacking parameters

% Target information
heading_desired = deg2rad(params.heading_desired); % desired heading for dive

% Aerodynamic surfaces properties
S = params.S; % wing surface area
cbar = params.cbar; % wing MAC
b = params.b; % wing span

% Glider properties

% Whole glider
Vol_static = params.Vstatic; % displaced volume w/o VBD
Mf = params.Mf; % 3x3 added mass matrix, should be changed later?
Jf = params.Jf; % 3x3 added mass inertia matrix, should be changed later?
ms = params.Ms; % stationary mass
Js = params.Js; % 3x3 stationary mass inertia matrix
VCB = -0.00362; % Vertical CB location relative to neutral axis, in m for SG 194

% Battery
VCG_bat = -0.01082; % Battery vertical cg location relative to neutral axis, in m for SG 194
% rpbat = sqrt( VCB^2 + VCG_bat^2 - 2 * VCB* VCG_bat * cosd(phi_bat) ); % distance from cb to cg of battery (in YZ plane)
mbat = params.mbat; % battery pack mass

% Kappa = 5.529e-06; % compressibility factor, number from Dr. Charlie Erikson Paper-- MAK
% tau = 7.05e-05; % Volumetric expansion, number from Dr. Charlie Erikson Paper -- MAK 
% PAPER: Assessing Seaglider Model-Based Position Accuracy on an Acoustic Tracking Range

% Ocean properties
rho = params.rho;  % density of ocean from CT 
% salt = params.salt; % salinity contesnt of water from sensor?
T0 = params.ambtemp; % ambient surface temp of water 
T = params.temp; % temperature
P = params.pressure; % pressure from ct sail?

% Unpacking coefficients
alphas = coefs.alphas; % 1 x M vector of alphas used in wind tunnel test
betas = coefs.betas; % 1 x N vector of betas used in wind tunnel test
CLs = coefs.CLs; % M x N array of lift coef values from wind tunnel test
Cds = coefs.CDs; % M x N array of drag coef values from wind tunnel test
Cys = coefs.CYs; % M x N array of side coef values from wind tunnel test
Crolls= coefs.Croll;% M x N array of roll coef values from wind tunnel test
Cpitchs=coefs.Cpitch;%MxN array of pitch coef values from wind tunnel test
Cyaws= coefs.Cyaw;% M x N array of yaw coef values from wind tunnel test

% Yaw angle
psi = heading - heading_desired;

% Battery position rp in body frame
rp = [x_bat; abs(VCG_bat) * sind(phi_bat) ; -VCB + abs(VCG_bat) * cosd(phi_bat) ];
skew = @(v) [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
rpx = skew(rp); % 3x3 cross product matrix for rp


%% Translational velocity, alpha, beta

% DCM for body frame to glide path frame
DCM_gb = angle2dcm(psi, theta, phi); % DCM glide to body
DCM_bg = DCM_gb.'; % DCM body to glide (it's just an inverse :) )

% Speed
V = sqrt(u^2+v^2+w^2); % norm function is for cowards

% Glide path and body frame velocity  
V_g = [u;v;w]; % glide path frame velocity
V_b = DCM_gb * V_g; % body frame velocity
ub = V_b(1);
vb = V_b(2);
wb = V_b(3);
% Note: We CAN implement the water currents in this part. Some frame 
%       silliness may be required.


% Flight path angles
V = sqrt(u^2 + v^2 + w^2);
if V > 1e-6
    alpha = atan2(wb, ub) * 180/pi;   % deg, body-frame angle of attack
    beta = asin(vb / V) * 180/pi;   % deg, body-frame sideslip
else
    alpha = 0;  beta = 0;
end
%% Mass and Inertia

% Effective translational and rotational inertia matrices
M = ms*eye(3) + Mf; % masses
J = Js + Jf ; % inertias

%% Forces & Coefs

% Dynamic Pressure
q = 0.5 * rho * V^2;

% Setting out of bounds angles to final angle in array (give em the clamps)
alpha_clamp = min(max(alpha, min(alphas)), max(alphas));
beta_clamp  = min(max(beta,  min(betas)),  max(betas));

% Coefs from lookup table
CL     = interp2(betas,alphas,CLs,beta_clamp,alpha_clamp,'linear');
CD     = interp2(betas,alphas,Cds,beta_clamp,alpha_clamp,'linear');
CY     = interp2(betas,alphas,Cys,beta_clamp,alpha_clamp,'linear');
Croll  = interp2(betas,alphas,Crolls,beta_clamp,alpha_clamp,'linear');
Cpitch = interp2(betas,alphas,Cpitchs,beta_clamp,alpha_clamp,'linear');
Cyaw   = interp2(betas,alphas,Cyaws,beta_clamp,alpha_clamp,'linear');


% Hydrodynamic forces in body frame 
L = q * CL * S; % lift
D = q * CD * S; % drag
Y = q * CY * S; % side
F_hydro = [-D; Y; -L];


% Buoyancy Force -- MAK


% Total displaced volume & mass
% VBD_ctlcc = VBD_ctlAD * -0.2453;    % converting VBD_ctl from AD to cm^3, VBD_ctlAD is control input
Vol_blad = (VBD_cc + 1426.7) * 1e-6;     % oil volume in bladder (m^3 now)
Vol_disp = Vol_blad + Vol_static;   % total volume displaced 
Volume = Vol_disp; %* exp(-(Kappa * P - tau * (T - T0))); % -- MAK
m_total = ms + mbat;       % total mass of the seaglider (ms will change with damage cases)
% m_disp = Vol_disp * rho;             % Not being used

% Gravity
g = 9.81;  % m/s^2

% Gravity in body frame
g_b = DCM_gb *[0;0;g];

% buoyancy equation 
B = g_b * (m_total - rho * Volume) ;


%% Moments (torques)

% Hydrodynamic moments (torques)
Mp = q * Croll * b * S; % roll moment
Mq = q * Cpitch * cbar * S; % pitch moment
Mr = q * Cyaw * b * S; % yaw moment
Torques = [Mp; Mq; Mr]; 

% Euler-rate vector in rad/s
eta_dot = [phidot; thetadot; headingdot]; %sensors give us euler angles
                                          %we'll convert for RB dynamics

% Kinematics matrix: eta_dot = H * omega, where omega = [p;q;r]
H = [1  sin(phi)*tan(theta)   cos(phi)*tan(theta);
     0  cos(phi)             -sin(phi);
     0  sin(phi)/cos(theta)   cos(phi)/cos(theta)];

% Time derivative of H
Hdot = [0,  cos(phi)*phidot*tan(theta) + sin(phi)*sec(theta)^2*thetadot, ...
            -sin(phi)*phidot*tan(theta) + cos(phi)*sec(theta)^2*thetadot;
        0, -sin(phi)*phidot, ...
            -cos(phi)*phidot;
        0,  cos(phi)*phidot*sec(theta) + sin(phi)*sec(theta)*tan(theta)*thetadot, ...
            -sin(phi)*phidot*sec(theta) + cos(phi)*sec(theta)*tan(theta)*thetadot];

% Body angular velocity
Omega = H \ eta_dot;   

%% Momentum (finally)

% Battery translational momentum
Pp = mbat * (V_b + cross(Omega, rp));

% Starting with vdot = M^-1 Fbar, see algebra in notes
% Feq = cross(M*V_b + Pp, Omega) + m0*g_b + F_hydro;
Feq = cross(M*V_b + Pp, Omega) + B + F_hydro;
veq = [M + mbat*eye(3), -mbat*rpx];

% Starting with Omegadot = J^-1 Tbar, see algebra in notes
Teq = cross(J*Omega + cross(rp, Pp), Omega) ...
      + cross(M*V_b, V_b) ...
      + mbat * cross(rp, g_b) ...
      + Torques;
Omegaeq = [mbat*rpx, J - mbat*rpx*rpx];

% Accelerations from equation in notes
accels = [veq ; Omegaeq] \ [Feq ; Teq];
Vdot_b = accels(1:3); % translational
Omegadot = accels(4:6); % angular

%% Dx output vector

% Body angular accel to Euler angular accel
eta_Ddot = Hdot * Omega + H * Omegadot ; % chain rule :)

% Body accel to glide accel
Vdot_g = DCM_bg * ( Vdot_b + cross(Omega,V_b) );
udot = Vdot_g(1);
vdot = Vdot_g(2);
wdot = Vdot_g(3);

% Converting rad/s back to deg/s
phiDdot = eta_Ddot(1) * 180/pi;
thetaDdot = eta_Ddot(2) * 180/pi;
headingDdot = eta_Ddot(3) * 180/pi;
phidot = phidot * 180/pi;
thetadot = thetadot * 180/pi;
headingdot = headingdot * 180/pi;

% Output
X_dyn = [phiDdot; thetaDdot; headingDdot; phidot; thetadot; headingdot; ...
         udot; vdot; wdot; u; v; w];

end

