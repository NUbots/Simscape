head = [-0.05482228998067195, -0.00010888238321339966, 0.0011712900228573724, 0.41900000000000004];
arm_upper = [-0.0004754647829298364, 0.0004830631532648927, -0.047851449636613815, 0.29700000000000004];
arm_lower = [0.05336545685394963, -0.0008156196838209869, 0.0006063260979097202, 0.26];
torso = [0.0050079703254699804, 0.008880921777563762, 0.1021503146237477, 2.5390000000000006];
hip_block = [0.01573294687984765, -0.0005421722579394279, 0.0012617565679100488, 0.306];
leg_upper = [-0.0868239473635843, -0.0009904680702090176, -0.0032236573318317092, 0.387];
leg_lower = [-0.10000847255111779, 0.0012849442073489777, -0.0070729066189781946, 0.177];
ankle_block = [-0.01573294687984541, 0.0005126082201406932, -0.02197623716321553, 0.306];
foot = [0.030807480494529724, 0.008771710644510001, 0.011298451871418577, 0.20500000000000002];

% com(HEAD_PITCH], head),
% com(L_SHOULDER_PITCH], arm_upper),
% com(R_SHOULDER_PITCH], arm_upper),
% com(L_SHOULDER_ROLL], arm_lower),
% com(R_SHOULDER_ROLL], arm_lower),
% com(L_HIP_ROLL], hip_block),
% com(R_HIP_ROLL], hip_block),
% com(L_HIP_PITCH], leg_upper),
% com(R_HIP_PITCH], leg_upper),
% com(L_KNEE], leg_lower),
% com(R_KNEE], leg_lower),
% com(L_ANKLE_PITCH], ankle_block),
% com(R_ANKLE_PITCH], ankle_block),
% com(L_ANKLE_ROLL], foot),
% com(R_ANKLE_ROLL], foot),



ms = [head; arm_upper; arm_upper; arm_lower; arm_lower; hip_block; hip_block; leg_upper; leg_upper; leg_lower; leg_lower; ankle_block; ankle_block; foot; foot; torso];
tf = [simout.HtHp.data(:,:,end), simout.HtRSp.data(:,:,end), simout.HtLSp.data(:,:,end), simout.HtRSr.data(:,:,end), simout.HtLSr.data(:,:,end), simout.HtRHr.data(:,:,end), simout.HtLHr.data(:,:,end), simout.HtRHp.data(:,:,end), simout.HtLHp.data(:,:,end), simout.HtRK.data(:,:,end), simout.HtLK.data(:,:,end), simout.HtRAp.data(:,:,end), simout.HtLAp.data(:,:,end), simout.HtRAr.data(:,:,end), simout.HtLAr.data(:,:,end), [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]];

% t = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]
% com = t * [torso(1:3) 1]'
% com = com(1:3)
% 
% t = simout.HtHp.data(:,:,end)
% com = [com;1] + t * [head(1:3) 1]'
% com = com(1:3)

% size(ms)
% size(tf)
% 
% ii = 1
% simout.HtHp.data(:,:,end)
% t = tf(1:4, ii*4-3:ii*4)
% 
% ii = 2
% simout.HtRSp.data(:,:,end)
% t = tf(1:4, ii*4-3:ii*4)

% Store CoM as x,y,z coordinates
com = [0;0;0];
mass = 0;

for ii=1:size(ms,1)
    % Get transform of the particle
    t = tf(1:4, ii*4-3:ii*4);
    % Alias particle CoM and mass for convenience
    p_com  = [ms(ii, 1:3) 1]';
    p_com  = t * p_com;
    p_mass = ms(ii, 4);
    % Calculate new CoM and updated mass
    com = (com * mass + p_com(1:3) * p_mass) / (mass + p_mass);
    mass = mass + p_mass;
end

com
mass

% for v  [head, arm_upper, arm_lower]
%     rtHeadm = Ht * head[1:3];
% end
