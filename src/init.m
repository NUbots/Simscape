clear all

%% Visual parameters
inertia_opacity = 0; % Determines how visible inertia indicators will be

%% Sample time
sample_time = 1/90;
g = 9.8107;

Ts = 1e-3;
sim_time = 6; % s Amount of time simulation runs for (maximum)

%% Define floor contact parameters
floor.length = 2;       % m (x)
floor.width = 2;        % m (y)
floor.depth = 0.02;     % m (z) 
floor.vel_gain = 1e-9;  
floor.contact_stiffness = 1e3;
floor.contact_damping = 1e3;
floor.kinetic_friction = 0.9;
floor.static_friction = 0.95;
floor.stud_radius = 1e-3;

%% Torso parameters
z0          = 0.48; % m (z)
Khr         = 0.03;  % Proportional gain for hip roll
Khp         = 0.01;  % Proportional gain for hip pitch
Ka          = 0.1;   % Proportional gain for ankle control

%% Initialise kinematics parameters
model = NUgusKinematics;

%% Initialise servos
MX64_init;
MX106_init;

%% Initialise CM740
CM740_init;
