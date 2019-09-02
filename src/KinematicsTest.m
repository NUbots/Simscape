clear all
clf
%% NUgus Kinematics Model
model = NUgusKinematics();
%% Create kinematics object
k = Kinematics(model);
npts = 150;
periods = 16*pi;
legOffset = round((0.25 * npts) / ((periods) / (2*pi)));
x = linspace(0, periods, npts+legOffset);
traj = sin(x) .^ 2;
figure(1)
plot(traj)

t = Transform3D();
% for time=1:npts/2
for time=1:npts-1
%     legOffset = round(periods * 10);
    
    
    rTRh = [0.05 + 0.05*traj(time+legOffset) -0.1 -0.1+0.1*traj(time+legOffset)];
    rTLh = [0.05 + 0.05*traj(time) 0.1 -0.1+0.01*traj(time+legOffset)];
    
    HTRl = t.translate([-0.1 + 0.1*traj(time) -0.03-0.05*traj(time) -0.45 + 0.1*traj(time)]);
    HTLl = t.translate([-0.1 + 0.1*traj(time+legOffset) 0.03+0.05*traj(time+legOffset) -0.45 + 0.1*traj(time+legOffset)]);
    rCP  = [1.0 0.5*traj(time) 0.0];

    [thetaTRSp, thetaTLSp, thetaTRSr, thetaTLSr, thetaTRE, thetaTLE, thetaTRHy, thetaTLHy, thetaTRHr, thetaTLHr, thetaTRHp, thetaTLHp, thetaTRK, thetaTLK, thetaTRAp, thetaTLAp, thetaTRAr, thetaTLAr, thetaTNy, thetaTNp] = k.getIK(rTRh, rTLh, HTRl, HTLl, rCP);
    %% For a zombie pose, calculate FK
    [HtRSp, HtLSp, HtRSr, HtLSr, HtRE, HtLE, HtRHy, HtLHy, HtRHr, HtLHr, HtRHp, HtLHp, HtRK, HtLK, HtRAp, HtLAp, HtRAr, HtLAr, HtHy, HtHp] = k.getFK([thetaTRSp, thetaTLSp, thetaTRSr, thetaTLSr, thetaTRE, thetaTLE, thetaTRHy, thetaTLHy, thetaTRHr, thetaTLHr, thetaTRHp, thetaTLHp, thetaTRK, thetaTLK, thetaTRAp, thetaTLAp, thetaTRAr, thetaTLAr, thetaTNy, thetaTNp]);
    fk = [HtRSp, HtLSp, HtRSr, HtLSr, HtRE, HtLE, HtRHy, HtLHy, HtRHr, HtLHr, HtRHp, HtLHp, HtRK, HtLK, HtRAp, HtLAp, HtRAr, HtLAr, HtHy, HtHp];

    figure(2)
    grid()
    % Scatter joints
    n = 4:4:4*20;
    scatter3([0, fk(1, n)],[0, fk(2, n)],[0, fk(3, n)]);
    hold on
    % Plot links
    % Head
    n = 4*19:4:4*20;
    plot3([0, fk(1, n)],[0, fk(2, n)],[0, fk(3, n)])
    % R Arm
    n = 4*2:4*2:4*6;
    plot3([0, fk(1, n)],[0, fk(2, n)],[0, fk(3, n)])
    % L Arm
    n = 4*1:4*2:4*5;
    plot3([0, fk(1, n)],[0, fk(2, n)],[0, fk(3, n)])
    % R Leg
    n = 4*7:4*2:4*17;
    plot3([0, fk(1, n)],[0, fk(2, n)],[0, fk(3, n)])
    % L Leg
    n = 4*8:4*2:4*18;
    plot3([0, fk(1, n)],[0, fk(2, n)],[0, fk(3, n)])

    set(gca, 'DataAspectRatio', [1 1 1]);
    xlim([-0.5 0.5])
    ylim([-0.5 0.5])
    zlim([-inf 0.5])
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    title('NUgus IK/FK')
%     view([time/npts 0.5 0.1])
    view([0.48 0.5 0.1])
    hold off
    
    pause(0.1)
end