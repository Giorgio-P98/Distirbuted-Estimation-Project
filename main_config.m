%% ENVIRONMENT CONSTRUCTION
w_t = 1.5;              % environment wall thickness
s = 50;                 % x-y map dimension [s[m]; s[m]]
n_pointxm = 1000;       % point x meter in obstacles generation

n_lidar = 360 ;         % n of ray in 2*pi rad of lidar sensing range

environment = 1;        % 1,2,3 for different environments (4 no obstacles)

%% SIMULATION INIT
t=0;                    % time t [s] init
i=0;                    % i var init for plot only at certain step
sim_t = 50;             % simulation time [s]
dt = 0.1;               % time step [s]
n_r = 7;                % Number of Agents
rs = 4;                 % Maximum Lidar measurements range
Rr = 0.25;              % Agents incumbrance radius
explored = 0;           % Explored fraction of total

grid_s = 1.0;           % grid cell dimension (mesh_map)
phi_max = 10;           % max value of the mesh_map (initial value)
n_verts = 41;           % Agents Voronoi cell verts number

% AGENTS POSITION
defined_pose = true;    % true if want to use pre-defined Agents pos,itions

% modify this vector to specify the birth position of the agents
robot_init = [s-50 0]+[48 9; 45 9; 42 9; 48 12; 45 12; 42 12; 45 15];
% robot_init = [s-50 0]+[8 23; 45 20; 45 45; 10 45; 45 12; 42 12; 45 15];

% Simulation noises
gps_n = 0.5;            % gps noise standard deviation
model_n = 0.05;         % model noise standard deviation
mag_n = 0.2;

% Point dynamic gain
kp = 8.0;               % proprotional control velocity gain

% Differential Dynamic gains
kg = 7.0;
kl = 1.5;

% general gain
kd = 1.0;               % dynamics set update mesh_map(Phi) gain
ku = 0.1;               % visited set update mesh_map(Phi) gain
k0 = 0.05;              % dynamics of the cell radius gain (decrease rate)
k1 = 0.95;              % dynamics of the cell radius gain (increase rate)

gains = {kp,kd,ku,k0,k1};
gains_ddr = {kg,kl,kd,ku,k0,k1};

% centroid update frequency
centroid_step = 1;      % every centroi_step, the centroid is updated 

% Clip value for maximum angular and linear velocity of the DDR
u_clip = 1000;
w_clip = 1000;

% WANT PLOT?
want_plot = true;          % true if you want dynamic plots
plot_step = 2;

% Environment polyshape
point_env = [0,0,s+10,s+10;0,s+5,s+5,0];
env = polyshape(point_env(1,:), point_env(2,:));

%% RENDEZVOUS

% rendezvous_yes = false;      % when true, the dynamic change to rendezvous
% rendezvous_pos = NaN(2,1);   % rendezvous point (x, y), initially unknown
% target_pt = NaN(2,1);        % target position (x, y), initially unknown

target_pos = [round(s/5); round(s/2)];
target_dim = 0.4;
ki = 2;
rho_i_init = 1.5;
rho_iD = 1;
discover_target = false;

rend_id = NaN;


