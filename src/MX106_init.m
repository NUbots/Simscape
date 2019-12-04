MX106_default = struct( ...
    'CW_angle_limit',int16(0), ...
    'CCW_angle_limit',int16(4095), ...
    'torque_enable',true, ...
    'D_gain',uint8(0), ...
    'I_gain',uint8(0), ...
    'P_gain',uint8(32), ...
    'goal_position',int16(0), ...
    'moving_speed',int16(1023), ...
    'torque_limit',int16(1023), ...
    'present_position',int16(0), ...
    'present_speed',int16(0), ...
    'present_load',int16(0), ...
    'moving',false, ...
    'pwm',int16(0));
MX106_bus = Simulink.Bus.createObject(MX106_default);

bus_name = MX106_bus.busName;
MX106_bus = eval(bus_name);
clear(bus_name);
clear('bus_name');

% Initialise limits for bus elements
for i=1:length(MX106_bus.Elements)
    element = MX106_bus.Elements(i);
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
    MX106_bus.Elements(i) = element;
end

clear element
clear i

% Set motor parameters according to Robotis documentation
% (http://emanual.robotis.com/docs/en/dxl/mx/mx-106/)
MX106_reduction = 255;                                %
MX106_max_speed = 55 * MX106_reduction / 60 * 2 * pi; % rad/s (128) % 55 (123)
MX106_stall_torque = 10.0 / MX106_reduction;          % Nm
MX106_stall_current = 6.3;                            % A
MX106_supply_voltage = 14.8;                          % V
MX106_position_gain = 4096 / 2 / pi;                  % rad (norm from 0 ... 4095)
MX106_speed_gain = 1024 / 8;                          % position difference (norm from -1023 ... 1023)
MX106_resistor = 25;                                 % Ohm
MX106_inductor = 6.37e-6;                             % H
MX106_Kv = 9.926e-3;                                  % Vrad/s
MX106_Kc = 8e-1;                                      % Nm/A [from measurements by T. Baroche
MX106_inertia = 9e-8 * MX106_reduction^2;             % kgm² from measurements
MX106_amplifier = MX106_supply_voltage/1024;          %
MX106_Te = 1/1000;                                    %
% MX106_Kp = 1/4;                                     %
MX106_Kp = 1/2;                                         % [Originally 1/4]  
MX106_Ki = 1/1024;                                    %
MX106_Kd = 1/8;                                       %  
MX106_Kl = 1024 / MX106_stall_torque;                 %
MX106_lub_friction = 0.05;                          % Nms/rad
% MX106_lub_friction = 8.49e-4;                         % Nms/rad
MX106_dry_friction = 0;                             % Nm
% MX106_dry_friction = 1.07;                            % Nm
MX106_init_position = 0;                              % rad
MX106_position_offset = pi;                           % rad (motor offset for zeroing)
MX106_init_speed = 0;                                 % rad/s
