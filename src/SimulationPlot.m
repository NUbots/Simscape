clf;

for ii=1:100:size(FK.HtRSp.data, 3)
    %% Get centre of mass in world space
    % Get Hwt and make homogeneous
    H = Hwt.data(:,:,ii);
    H(4,4) = 1;
    % Pack the world-space Forward Kinematics homogeneous matrices
    fk = [H * FK.HtRSp.data(:,:,ii), H * FK.HtLSp.data(:,:,ii), H * FK.HtRSr.data(:,:,ii), H * FK.HtLSr.data(:,:,ii), H * FK.HtRE.data(:,:,ii), H * FK.HtLE.data(:,:,ii), H * FK.HtRHy.data(:,:,ii), H * FK.HtLHy.data(:,:,ii), H * FK.HtRHr.data(:,:,ii), H * FK.HtLHr.data(:,:,ii), H * FK.HtRHp.data(:,:,ii), H * FK.HtLHp.data(:,:,ii), H * FK.HtRK.data(:,:,ii), H * FK.HtLK.data(:,:,ii), H * FK.HtRAp.data(:,:,ii), H * FK.HtLAp.data(:,:,ii), H * FK.HtRAr.data(:,:,ii), H * FK.HtLAr.data(:,:,ii), H * FK.HtHy.data(:,:,ii), H * FK.HtHp.data(:,:,ii)];
%     fk = [FK.HtRSp.data(:,:,ii) * H, FK.HtLSp.data(:,:,ii) * H, FK.HtRSr.data(:,:,ii) * H, FK.HtLSr.data(:,:,ii) * H, FK.HtRE.data(:,:,ii) * H, FK.HtLE.data(:,:,ii) * H, FK.HtRHy.data(:,:,ii) * H, FK.HtLHy.data(:,:,ii) * H, FK.HtRHr.data(:,:,ii) * H, FK.HtLHr.data(:,:,ii) * H, FK.HtRHp.data(:,:,ii) * H, FK.HtLHp.data(:,:,ii) * H, FK.HtRK.data(:,:,ii) * H, FK.HtLK.data(:,:,ii) * H, FK.HtRAp.data(:,:,ii) * H, FK.HtLAp.data(:,:,ii) * H, FK.HtRAr.data(:,:,ii) * H, FK.HtLAr.data(:,:,ii) * H, FK.HtHy.data(:,:,ii) * H, FK.HtHp.data(:,:,ii)];
    % Create a transform from the centre of mass location in torso space
    Htc = Transform3D().translate(com.data(:,:,ii));
    % Get centre of mass in world space
    Hwc = H * Htc.tf;
    % Calculate world-space centre of mass
    com_ii = Hwc(1:3, 4);
    
    %% Plot
    figure(1)
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
    
    % Plot COM and its projection
    scatter3(com_ii(1), com_ii(2), com_ii(3), 'x');
    scatter3(com_ii(1), com_ii(2), H(4,3), 'x');
    plot3([com_ii(1), com_ii(1)], [com_ii(2), com_ii(2)], [com_ii(3), H(4,3)], '--');
    
    % Plot ground surface
    Xp = [floor.length/2, floor.length/2, -floor.length/2, -floor.length/2];
    Yp = [floor.width/2, -floor.width/2, -floor.width/2, floor.width/2];
    Zp = [H(4,3), H(4,3), H(4,3), H(4,3)];
    fill3(Xp, Yp, Zp, [0.9, 0.9, 0.9]);
    
    % Set plot parameters
    set(gca, 'DataAspectRatio', [1 1 1]);
    xlim([-0.5 0.5])
    ylim([-0.5 0.5])
    zlim([-inf 0.5])
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    title('NUgus IK/FK')
    view([0.48 0.5 0.1])
    hold off

    % saveas(gcf,sprintf('f%d.png', time))
    pause(0.001)
end