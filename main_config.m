%% ENVIRONMENT CONSTRUCTION
w_t = 1.5;              % environment wall thickness
s = 50;                 % x-y map dimension [s[m]; s[m]]
n_pointxm = 500;        % point x meter in obstacles generation(for lidar)

environment = 1;        % 1, 2 or 3 for different environments

%% SIMULATION INIT
t=0;                    % time t [s] init
i=0;                    % i var init for plot only at certain step
sim_t = 50;             % simulation time [s]
dt = 0.1;               % time step [s]
n_r = 7;                % Number of Agents
rs = round(s/13,1);     % Maximum Lidar measurements range
Rr = 0.25;              % Agents incumbrance radius
explored = 0;           % Explored fraction of total

grid_s = 0.5;           % grid cell dimension (mesh_map)
phi_max = 10;           % max value of the mesh_map (initial value)
n_verts = 31;           % Agents Voronoi cell verts number

% AGENTS POSITION
defined_pose = true;    % true if want to use pre-defined Agents pos,itions

% modify this vector to specify the birth position of the agents
robot_init = [s-50 0]+[48 9; 45 9; 42 9; 48 12; 45 12; 42 12; 45 15];

% Simulation noises
gps_n = round(0.01*rs/10,3);          % gps noise standard deviation
model_n = round(0.01*rs/10,3);        % model noise standard deviation

% Algo gains
kp = 10;                 % proprotional control velocity gain
kd = 1.0;               % dynamics set update mesh_map(Phi) gain
ku = 0.2;               % visited set update mesh_map(Phi) gain
k0 = 1.5;               % dynamics of the cell radius gain (decrease rate)
k1 = 0.3;               % dynamics of the cell radius gain (increase rate)

gains = {kp,kd,ku,k0,k1};

% WANT PLOT?
want_plot = false;      % true if you want dynamic plot

% Environment polyshape

point_env = [0,0,s+10,s+10;0,s+5,s+5,0];
env = polyshape(point_env(1,:), point_env(2,:));


