% Define floor contact parameters
floor.length = 2;       % m (x)
floor.width = 2;        % m (y)
floor.depth = 0.01;     % m (z)
floor.pos_gain = -1e5;  % N/m 
floor.vel_gain = -1e3;  % Ns/m
floor.friction = 0.6;   % Friction coeff.

% Initialise servos
MX64_init;
MX106_init;
% Initialise CM740
CM740_init;