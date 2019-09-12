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
    % Masses
    model.mass.head = [-0.05482228998067195, -0.00010888238321339966, 0.0011712900228573724, 0.41900000000000004];
    model.mass.arm_upper = [-0.0004754647829298364, 0.0004830631532648927, -0.047851449636613815, 0.29700000000000004];
    model.mass.arm_lower = [0.05336545685394963, -0.0008156196838209869, 0.0006063260979097202, 0.26];
    model.mass.torso = [0.0050079703254699804, 0.008880921777563762, 0.1021503146237477, 2.5390000000000006];
    model.mass.hip_block = [0.01573294687984765, -0.0005421722579394279, 0.0012617565679100488, 0.306];
    model.mass.leg_upper = [-0.0868239473635843, -0.0009904680702090176, -0.0032236573318317092, 0.387];
    model.mass.leg_lower = [-0.10000847255111779, 0.0012849442073489777, -0.0070729066189781946, 0.177];
    model.mass.ankle_block = [-0.01573294687984541, 0.0005126082201406932, -0.02197623716321553, 0.306];
    model.mass.foot = [0.030807480494529724, 0.008771710644510001, 0.011298451871418577, 0.20500000000000002];
end