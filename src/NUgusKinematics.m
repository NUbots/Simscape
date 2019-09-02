function model = NUgusKinematics()
    %% NUgus Kinematics Model
    % Leg
    model.leg.HIP_OFFSET = [0.00, 0.055, 0.045]; % [x, y, z]
    model.leg.UPPER_LEG_LENGTH = 0.2;
    model.leg.LOWER_LEG_LENGTH = 0.2;
    model.leg.HEEL_LENGTH = 0.085; % measured
    model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE = 0.02; % rough
    model.leg.foot.WIDTH = 0.130; % rough
    model.leg.foot.HEIGHT = 0.04;
    model.leg.foot.LENGTH = 0.215; % rough
    model.leg.foot.TOE_LENGTH = 0.13; % measured
    model.leg.left_to_right.HIP_YAW =    -1;
    model.leg.left_to_right.HIP_ROLL =   -1;
    model.leg.left_to_right.HIP_PITCH =   1;
    model.leg.left_to_right.KNEE =        1;
    model.leg.left_to_right.ANKLE_PITCH = 1;
    model.leg.left_to_right.ANKLE_ROLL = -1;
    % Head
    model.head.CAMERA_DECLINATION = pi / 90.0; % 2 degrees
    model.head.NECK_TO_CAMERA = [0.069, 0, 0.065]; % [x, y, z]
    model.head.IPD = 0.068; % Interpupillary distance - y axis
    model.head.neck.LENGTH = 0.048;
    model.head.neck.POSITION = [-0.007, 0, 0.21]; % [x, y, z]
    model.head.limits.YAW = [-pi/4, pi/4]; % [min_yaw, max_yaw]
    model.head.limits.PITCH = [pi/6, pi/6]; % [min_pitch, max_pitch]
    % Arm
    model.arm.DISTANCE_BETWEEN_SHOULDERS = 0.17; % distance between shoulders
    model.arm.shoulder.DIMS = [0.0 0.04 0.010]; % length, width, height
    model.arm.shoulder.OFFSET = [0.0, 0.1905]; % [x, z]
    model.arm.upper_arm.LENGTH = 0.16;
    model.arm.upper_arm.OFFSET = [0, 0.03]; % [y, x] very rough
    model.arm.lower_arm.LENGTH = 0.235;
    model.arm.lower_arm.OFFSET = [0, 0]; % [y, z] very rough
end