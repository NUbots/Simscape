MX64_default = struct( ...
    'CW_angle_limit',int16(0), ...
    'CCW_angle_limit',int16(4095), ...
    'torque_enable',true, ...
    'D_gain',uint8(0), ...
    'I_gain',uint8(0), ...
    'P_gain',uint8(32), ...
    'goal_position',int16(2048), ...
    'moving_speed',int16(0), ...
    'torque_limit',int16(1023), ...
    'present_position',int16(0), ...
    'present_speed',int16(0), ...
    'present_load',int16(0), ...
    'moving',false, ...
    'pwm',int16(0));
MX64_bus = Simulink.Bus.createObject(MX64_default);

bus_name = MX64_bus.busName;
MX64_bus = eval(bus_name);
clear(bus_name);
clear('bus_name');

% Initialise limits for bus elements
for i=1:length(MX64_bus.Elements)
    element = MX64_bus.Elements(i);
    if strcmp(element.Name,'CW_angle_limit')
        element.Description = 'Clockwise Angle Limit';
        element.Min = 0;
        element.Max = 4095;
    elseif strcmp(element.Name,'CCW_angle_limit')
        element.Description = 'Counter-Clockwise Angle Limit';
        element.Min = 0;
        element.Max = 4095;
    elseif strcmp(element.Name,'torque_enable')
        element.Description = 'Torque On/Off';
        element.Min = 0;
        element.Max = 1;
    elseif strcmp(element.Name,'D_gain')
        element.Description = 'Derivative Gain';
        element.Min = 0;
        element.Max = 255;
    elseif strcmp(element.Name,'I_gain')
        element.Description = 'Integral Gain';
        element.Min = 0;
        element.Max = 255;
    elseif strcmp(element.Name,'P_gain')
        element.Description = 'Proportional Gain';
        element.Min = 0;
        element.Max = 255;
    elseif strcmp(element.Name,'goal_position')
        element.Description = 'Goal Position';
        element.Min = 0;
        element.Max = 4095;
    elseif strcmp(element.Name,'moving_speed')
        element.Description = 'Moving Speed';
        element.Min = 0;
        element.Max = 1023;
    elseif strcmp(element.Name,'torque_limit')
        element.Description = 'Torque Limit';
        element.Min = 0;
        element.Max = 1023;
    elseif strcmp(element.Name,'present_position')
        element.Description = 'Present Position';
        element.Min = 0;
        element.Max = 4095;
    elseif strcmp(element.Name,'present_speed')
        element.Description = 'Present Speed';
        element.Min = -1023;
        element.Max = +1023;
    elseif strcmp(element.Name,'present_load')
        element.Description = 'Present Load';
        element.Min = -1023;
        element.Max = +1023;
    elseif strcmp(element.Name,'moving')
        element.Description = 'Moving';
        element.Min = 0;
        element.Max = 1;
    elseif strcmp(element.Name,'pwm')
        element.Description = 'pwm';
        element.Min = -1023;
        element.Max = +1023;
    else
        error('[ERR] Invalid element name "%s"',element.Name);
    end
    MX64_bus.Elements(i) = element;
end

clear element
clear i

% Set motor parameters according to Robotis documentation
% (http://emanual.robotis.com/docs/en/dxl/mx/mx-64/)
MX64_reduction = 200.0;                             % 
MX64_max_speed = 78 * MX64_reduction / 60 * 2 * pi; % rad/s
MX64_stall_torque = 6.0 / MX64_reduction;           % Nm
MX64_stall_current = 5.2;                           % A
MX64_supply_voltage = 14.8;                         % V
% MX64_position_gain = 4096 / 2 / pi;                 % rad (norm from 0 ... 4095)
MX64_position_gain = 2048 / 2 / pi;                 % rad (norm from 0 ... 4095)
MX64_speed_gain = 1024 / 8;                         % position difference (norm from -1023 ... 1023)
MX64_resistor = 500;                                % Ohm
MX64_inductor = 6.37e-3;                            % H
MX64_Kv = 9.926e-3;                                 % Vrad/s
MX64_Kc = 8e-1;                                     % Nm/A [from measurements by T. Baroche]
MX64_inertia = 9e-8 * MX64_reduction^2;             % kgm² from measurements
MX64_amplifier = MX64_supply_voltage/1024;          % 
MX64_Te = 1/1000;                                   % 
MX64_Kp = 1/4;                                      % 
MX64_Ki = 1/1024;                                   % 
MX64_Kd = 1/8;                                      % 
MX64_Kl = 1024 / MX64_stall_torque;                 %
% MX64_lub_friction = 0.05;                           % Nms/rad
MX64_lub_friction = 8.49e-4;                       % Nms/rad
% MX64_dry_friction = 0;                              % Nm
MX64_dry_friction = 1.07;                          % Nm
MX64_init_position = 0;                             % rad
MX64_init_speed = 0;                                % rad/s
