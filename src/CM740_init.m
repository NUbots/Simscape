CM740_default = struct(...
    'acc_x',  int16(512),...
    'acc_y',  int16(512),...
    'acc_z',  int16(512),...
    'gyro_x', int16(512),...
    'gyro_y', int16(512),...
    'gyro_z', int16(512)...
);
CM740_bus = Simulink.Bus.createObject(CM740_default);

bus_name = CM740_bus.busName;
CM740_bus = eval(bus_name);
clear(bus_name);
clear('bus_name');

% Initialise limits for bus elements
for i=1:length(CM740_bus.Elements)
    element = CM740_bus.Elements(i);
    if strcmp(element.Name,'acc_x')
        element.Description = 'Accelerometer x-axis';
        element.Min = 0;
        element.Max = 1023;
    elseif strcmp(element.Name,'acc_y')
        element.Description = 'Accelerometer y-axis';
        element.Min = 0;
        element.Max = 1023;
    elseif strcmp(element.Name,'acc_z')
        element.Description = 'Accelerometer z-axis';
        element.Min = 0;
        element.Max = 1023;
    elseif strcmp(element.Name,'gyro_x')
        element.Description = 'Gyroscope x-axis';
        element.Min = 0;
        element.Max = 1023;
    elseif strcmp(element.Name,'gyro_y')
        element.Description = 'Gyroscope y-axis';
        element.Min = 0;
        element.Max = 1023;
    elseif strcmp(element.Name,'gyro_z')
        element.Description = 'Gyroscope z-axis';
        element.Min = 0;
        element.Max = 1023;
    else
        error('[ERR] Found invalid element name: "%s"',element.Name);
    end
    CM740_bus.Elements(i) = element;
end

clear element
clear i

% Set IMU parameters according to Robotis documentation
% (http://emanual.robotis.com/docs/en/platform/op2/getting_started/#sub-controllercm-740)
CM740_acc_bias    = 4 * 9.80743;             % acceleration bias value (from MRL)
CM740_acc_gain    = 512 / CM740_acc_bias;  % m/s^2
CM740_gyro_bias   = 1600 / 360 * 2 * pi;     % gyro bias value (from MRL)
CM740_gyro_gain   = 512 / CM740_gyro_bias; % rad/s
CM740_sample_time = 1 / 90;