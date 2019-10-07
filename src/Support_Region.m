close all

%% Plot foot space and world space for the feet
figure()
hold on
    pl = getSupport(FK.HtRAr.data(:,:,end), Htw.data(:,:,end), 1);
    pr = getSupport(FK.HtLAr.data(:,:,end), Htw.data(:,:,end), 2);
hold off

%% Plot support polygons
figure()
hold on
    plotP(pl, pr);
    % Plot CoM and ZMP
    c = Htw.data(:,:,end) * [com.data(:,1:3,end) 1]';
    z = Htw.data(:,:,end) * [zmp.data(end,1:3) 1]';
    
    scatter(c(1), c(2), 'x');
    text(c(1) + 0.009, c(2), 'CoM');
    scatter(z(1), z(2), 'x');
    text(z(1) + 0.009, z(2), 'ZMP');
hold off
    
%% Get the feet positions in foot and world space
function p = getSupport(HtF, H, n)
    model = NUgusKinematics();

    % Calculate transforms for each foot point
    F_tf = Transform3D();
    F_tf.tf = HtF;
    F_tf = F_tf.translateY(model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE);

    t = [model.leg.foot.LENGTH / 2; model.leg.foot.WIDTH / 2; 0; 1];
    HfFl = F_tf.translate(t .* [1; 1; 0; 1]);
    HfFr = F_tf.translate(t .* [1; -1; 0; 1]);
    HfBl = F_tf.translate(t .* [-1; 1; 0; 1]);
    HfBr = F_tf.translate(t .* [-1; -1; 0; 1]);
    
    subplot(1,2,1)
    hold on
        title('Torso Space')
        view([0 0 1])
        set(gca, 'DataAspectRatio', [1 1 1]);
        xlabel('x')
        ylabel('y')
        zlabel('z')

        plotT(F_tf.tf, true)
        plotT(HfFl.tf, false)
        plotT(HfFr.tf, false)
        plotT(HfBl.tf, false)
        plotT(HfBr.tf, false)
    hold off

    % Get transforms to feet extrema in world space
    HwFl = Transform3D(H * HfFl.tf);
    HwFr = Transform3D(H * HfFr.tf);
    HwBl = Transform3D(H * HfBl.tf);
    HwBr = Transform3D(H * HfBr.tf);
       
    subplot(1,2,2)
    hold on
        title('World Space')
        view([0 0 1])
        set(gca, 'DataAspectRatio', [1 1 1]);
        xlabel('x')
        ylabel('y')
        zlabel('z')
        plotT(H * F_tf.tf, true)
        plotT(HwFl.tf, false)
        plotT(HwFr.tf, false)
        plotT(HwBl.tf, false)
        plotT(HwBr.tf, false)
    hold off
    
    p = [HwFl.tf(1:3,4) HwFr.tf(1:3,4) HwBr.tf(1:3,4) HwBl.tf(1:3,4) HwFl.tf(1:3,4)];
end

%% Transform plot
function [] = plotT(T, plotAxes)
    % Get origin in T space
    v0 = T * [0;0;0;1];    
    % Get axes in T space
    vx = T * [1;0;0;0];
    vy = T * [0;1;0;0];
    vz = T * [0;0;1;0];
    
    if plotAxes 
        hold on
            grid on
            quiver3(v0(1),v0(2),v0(3), vx(1), vx(2), vx(3), 'r')
            quiver3(v0(1),v0(2),v0(3), vy(1), vy(2), vy(3), 'g')
            quiver3(v0(1),v0(2),v0(3), vz(1), vz(2), vz(3), 'b')
        hold off
    else
        hold on
            scatter3(v0(1), v0(2), v0(3))
        hold off
    end
end

%% Projection plot
function [] = plotP(pl, pr)
    hold on
        grid on
        set(gca, 'DataAspectRatio', [1 1 1]);
        % Left support
        lmid = pl(1:2,1) + 0.5 * (pl(1:2, 3) - pl(1:2,1));
        plot(pl(1,:), pl(2,:));
        scatter(lmid(1), lmid(2));
        % Right support
        rmid = pr(1:2,1) + 0.5 * (pr(1:2, 3) - pr(1:2,1));
        plot(pr(1,:), pr(2,:));
        scatter(rmid(1), rmid(2));
        % Plot midpoint between supports
        smid = lmid + 0.5 * (rmid - lmid);
        scatter(smid(1), smid(2));
end

