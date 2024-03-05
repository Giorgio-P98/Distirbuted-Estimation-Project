%% ENVIRONMENT CONSTRUCTION
w_t = 1.5;              % environment wall thickness
s = 50;                 % x-y map dimension [s[m]; s[m]]
n_pointxm = 1000;          % point x meter in obstacles generation
n_pointxm_meas = 2000;       % point x meter in obstacles generation when detected

n_lidar = 360 ;         % n of ray in 2*pi rad of lidar sensing range

environment = 1;        % 1,2,3 for different environments (4 no obstacles)

t_explo = 30;


%% SIMULATION INIT
t=0;                    % time t [s] init
i=0;                    % i var init for plot only at certain step
sim_t = 100;             % simulation time [s]
dt = 0.1;               % time step [s]
n_r = 5;                % Number of Agents
rs = 4.0;               % Maximum Lidar measurements range
Rr = 0.25;              % Agents incumbrance radius
explored = 0;           % Explored fraction of total
explored2 = 0;
explor_limit = 0.95;    % Environment is fully explored
explored_set = polyshape();

grid_s = 0.8;           % grid cell dimension (mesh_map)
phi_max = 10;           % max value of the mesh_map (initial value)
n_verts = 41;           % Agents Voronoi cell verts number

% AGENTS POSITION
defined_pose = true;    % true if want to use pre-defined Agents pos,itions

% Concavity threshold
conc_th = -0.05;

% modify this vector to specify the birth position of the agents
robot_init = [s-50 0]+[50 8; 45 8; 42 13; 47 13; 53 13; 50 17; 45 17];
% robot_init = [s-50 0]+[8 23; 45 20; 45 45; 10 45; 45 12; 42 12; 45 15];

% Simulation noises
gps_n = 0.5;           % gps noise standard deviation
model_n = 0.1;         % model noise standard deviation
mag_n = 2;             % magnetometer noise standard deviation
lidar_rad_std = 0.05;  % lidar radial noise std

% Point dynamic gain
kp = 8.0;               % proprotional control velocity gain

% Differential Dynamic gains
kg = 3;
kl = 0.05;
% general gain
kd = 1.7;              % dynamics set update mesh_map(Phi) gain
ku = 0.2;             % visited set update mesh_map(Phi) gain
k0 = 0.0;              % dynamics of the cell radius gain (decrease rate) 0.05
k1 = 0.0;              % dynamics of the cell radius gain (increase rate) 0.95

gains = {kp,kd,ku,k0,k1};
% gains_ddr = {kg,kl,kd,ku,k0,k1};
gains_ddr = {kg,kl,kd,ku};

% centroid update frequency
centroid_step = 1;      % every centroi_step, the centroid is updated 

% Clip value for maximum angular and linear velocity of the DDR
u_clip = 1000;
w_clip = 1000;

% WANT PLOT?
want_plot = false;          % true if you want dynamic plots
plot_step = 1;

% Environment polyshape
point_env = [0,0,s+10,s+10;0,s+5,s+5,0];
env = polyshape(point_env(1,:), point_env(2,:));

%% RENDEZVOUS
REND = false;

target_pos = [round(s/5); round(s/2)];
target_dim = 0.4;
ki = 2;
rho_i_init = 1.5;
rho_iD = 1;
discover_target = false;

rend_id = NaN;

%% ENVIRONMENT CONSTRUCTION

if environment == 1
    % Environment 1
    n_obs = 7;
    obs1 = [5;5]+[0,0,s,s,s+5,s+5,-5,-5;0,s,s,0,0,s+5,s+5,0];
    obs2 = [5;0]+[-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [5;0]+[2*s/3,2*s/3,2*s/3+w_t,2*s/3+w_t;5,s/3+3,s/3+3,5];
    obs4 = [5;5]+[s,2*s/3,2*s/3,2*s/3+w_t,2*s/3+w_t,s;s/2,s/2,4*s/5,4*s/5,s/2+w_t,s/2+w_t];
    obs5 = [5;0]+[-2;0]+[s/2,s/2,s/2+w_t,s/2+w_t;5,4*s/5,4*s/5,5];
    obs6 = [5;0]+[-3;0]+[s/3+w_t,s/3+w_t,s/5,s/5,s/3,s/3;s+5,2*s/3,2*s/3,2*s/3+w_t,2*s/3+w_t,s+5];
    obs7 = [5;0]+[0,s/3,s/3,0;s/3+w_t,s/3+w_t,s/3,s/3];
    obsall = {obs1,obs2,obs3,obs4,obs5,obs6,obs7};
elseif environment == 2
    % Environment Two
    door = 6;
    corridor = 5;
    n_obs = 6;
    obs1 = [5;5]+[0,0,s,s,s+5,s+5,-5,-5;0,s,s,0,0,s+5,s+5,0];
    obs2 = [5;0]+[-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [5;5]+[corridor,corridor,s/3,s/3,corridor+w_t,corridor+w_t;0,s/3,s/3,s/3-w_t,s/3-w_t,0];
    obs4 = [5;5]+[s/3+door,2*s/3+w_t,2*s/3+w_t,2*s/3,2*s/3,s/3+door;s/3,s/3,0,0,s/3-w_t,s/3-w_t];
    obs5 = [5;2]+[0;s/3+corridor]+[0,2*s/3+w_t,2*s/3+w_t,s/3+w_t + door/2,s/3+w_t + door/2,2*s/3,2*s/3,0;s/3+7,s/3+7,5+0.7,5+0.7,5+w_t+0.7,5+w_t+0.7,s/3-w_t+7,s/3-w_t+7];
    obs6 = [5;2.7]+[0;s/3+corridor]+[0,s/3+w_t - door/2,s/3+w_t - door/2,0;5,5,5+w_t,5+w_t];
    obsall = {obs1,obs2,obs3,obs4,obs5,obs6};
elseif environment == 3
    % Environment 3 
    n_obs = 4;
    obs1 = [5;5]+[0,0,s,s,s+5,s+5,-5,-5;0,s,s,0,0,s+5,s+5,0];
    obs2 = [5;0]+[-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [5;0]+[0.15*s,0.3*s,0.45*s,0.5*s,0.3*s;0.2*s,0.2*s,0.3*s,0.6*s,0.9*s];
    obs4 = [5;0]+[0.6*s,0.8*s,0.8*s,0.6*s;0.5*s,0.5*s,0.8*s,0.8*s];
    obsall = {obs1,obs2,obs3,obs4};
elseif environment == 4
    % Environment 4 (no obstacles)
    n_obs = 2;
    obs1 = [5;5]+[0,0,s,s,s+5,s+5,-5,-5;0,s,s,0,0,s+5,s+5,0];
    obs2 = [5;0]+[-5,s+5,s+5,-5;5,5,0,0];
    obsall = {obs1,obs2};
else
    disp('There are maximum 4 Environment');
end


