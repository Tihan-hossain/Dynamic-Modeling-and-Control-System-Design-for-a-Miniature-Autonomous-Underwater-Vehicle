clear;

% Mass Properties
m = 2.24; % kg
Cg = [0, 0, 0.021]; % Center of gravity 
%Cg = [0.001, -0.001, -0.01]; % Center of gravity 
Bc = [0, 0, 0.042]; % Center of Buoyancy 

% Moments of Inertia (kg.m^2)
Ixx = 0.028; Iyy = 0.028; Izz = 0.003;
% Ixy = -2.229e-5; Ixz = -5.178e-6; Iyz = 2.465e-5;
Ixy = 0; Ixz = 0; Iyz = 0;
I = [Ixx,Iyy, Izz, Ixy, Ixz, Iyz];

%AUV Geometry
d = 0.0889; % Diameter (m)
L = 0.47; % Length (m)
R = d / 2; % Radius (m)
V = 0.001; % Displaced volume (m³)
rho = 1025; % Seawater density (kg/m³)


% Added Mass Coefficients
Ca_x = 0.1; % Surge
Ca_yz = 0.9; % Sway & Heave
Ci_x = 0.2; % Roll
Ci_yz = 0.2; % Pitch & Yaw
C_off = 0.08; % Off-diagonal coupling coefficient
d_offset = 0.05; % Offset distance

% Added Mass Terms
m_ax = Ca_x * rho * V;
m_ay = Ca_yz * rho * V;
m_az = Ca_yz * rho * V;

% Added Moments of Inertia
I_ax = 0;
I_ay = rho * (pi/8) * (((L/2)^2 - R^2)^2);
I_az = rho * (pi/8) * (((L/2)^2 - R^2)^2);

% Off-Diagonal Terms
m_off = C_off * rho * V * d_offset;

% Added Mass Matrix (Ma) Axisymmetric with respect to X-axis  
Ma = [
    m_ax  0    0   0     0    0;
    0   m_ay   0   0     0  m_off;
    0     0   m_az 0   -m_off  0;
    0     0    0  0   0    0;
    0     0  -m_off 0    I_ay  0;
    0   m_off  0   0     0    I_az
];


% Body Mass Matrix 
Mb = [
    m  0  0  0  m*Cg(3)  0;
    0  m  0  -m*Cg(3)  0  0;
    0  0  m  0  0  0;
    0  -m*Cg(3)  0  Ixx  0  -Ixz;
    m*Cg(3)  0  0  0  Iyy  0;
    0  0  0   -Ixz 0  Izz;
];


Mt = Mb + Ma;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Quadratic Damping Coefficients(No Bouyancy module)
 % Xu_nl =  0.8995; % Surge
 % Yv_nl = 4.939; % Sway
 % Zw_nl = 4.939; % Heave
 % Kp_nl = 0.12; % Roll
 % Mq_nl = 0.31; % Pitch
 % Nr_nl = 0.31; % Yaw

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Quadratic Damping Coefficients(with Bouyancy module)
 Xu_nl =  2.320; % Surge
 Yv_nl = 16.860; % Sway
 Zw_nl = 16.860; % Heave
 Kp_nl = 1.710; % Roll
 Mq_nl =  12.407; % Pitch
 Nr_nl =  12.407; % Yaw

%Quadratic Damping Coefficients with Fins
% Xu_nl =  2.167; % Surge
%  Yv_nl = 15.48; % Sway
%  Zw_nl = 15.48; % Heave
%  Kp_nl = 0.442; % Roll
%  Mq_nl = 0.704; % Pitch
%  Nr_nl = 0.704; % Yaw

 %Quadratic Damping Matrix
 DNL = diag([Xu_nl, 0, 0, 0, 0, 0; 
                0, Yv_nl, 0, 0, 0, 0; 
                0, 0, Zw_nl, 0, 0, 0; 
                0, 0, 0, Kp_nl, 0, 0; 
                0, 0, 0, 0, Mq_nl, 0; 
                0, 0, 0, 0, 0, Nr_nl]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % % Linear Damping Coefficients(Bouyancy module)
  Xu = 0.552; % Surge
  Yv = 0.176; % Sway
  Zw = 0.176; % Heave
  Kp = 0.681; % Roll
  Mq = 0.219 ; % Pitch
  Nr = 0.219 ; % Yaw 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % % Linear Damping Coefficients(No Bouyancy module)
  % Xu = 0.03; % Surge
  % Yv = 0.365; % Sway
  % Zw = 0.365; % Heave
  % Kp = 0.015; % Roll
  % Mq = 0.026; % Pitch
  % Nr = 0.026; % Yaw 

   % Linear Damping Coefficients (DL) with fins 
  % Xu = 0.068; % Surge
  % Yv = 0.2103; % Sway
  % Zw = 0.2103; % Heave
  % Kp = 0.1361; % Roll
  % Mq = 0.099; % Pitch
  % Nr = 0.099; % Yaw
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %  Linear Damping Matrix
  DL = diag([Xu, 0, 0, 0, 0, 0; 
               0, Yv, 0, 0, 0, 0; 
               0, 0, Zw, 0, 0, 0; 
               0, 0, 0, Kp, 0, 0; 
               0, 0, 0, 0, Mq, 0; 
               0, 0, 0, 0, 0, Nr]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference model
zeta = 1.5; %1.2
Omega = [0.085; 0.085; 0.9];
Delta = Omega / 2*zeta;
Delta_2 = Delta.^2;

Af = [0.5; 0.1; 0.5];