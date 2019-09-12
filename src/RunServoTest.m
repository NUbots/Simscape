% init;
% sim("ServoTest");

clf;

% ServoData = readtable("MX106_Sawtooth.csv");
% ServoData = readtable("MX106_Sinewave.csv");
ServoData = readtable("MX106_Square.csv");

% ServoData = readtable("MX64_Sawtooth.csv");
% ServoData = readtable("MX64_Sinewave.csv");
% ServoData = readtable("MX64_Square.csv");

r = mean(abs(ServoData.PresentSpeed_rad_s_))
s = mean(abs(present_speed.data))

gain = r / s

% Alias colours for convenience
blue = [0 0.5 0.7];
orange = [1 0.5 0];
green = [0 0 0 0.3];
% Alias legend location for convenience
legend_loc = 'northeast';

ax1 = subplot(3,1,1);
grid('on')
ax = gca;
ax.GridLineStyle = '--';
hold on
    plot(ServoData.PresentPosition_rad_, 'Color', blue);    
    plot(present_position, '-.', 'Color', orange);
    plot(goal_position, ':', 'Color', green);
    legend("Real", "Sim.", "Goal", 'Location', legend_loc);
    ylabel("Position (rad)");
    title("Square Wave Trajectory (MX106)");
    xlabel("");
    yticks([0 pi 2*pi]);
    yticklabels(["0" "\pi" "2\pi"]);
    axis([0 300 0 2*pi])
hold off


ax2 = subplot(3,1,2);
grid('on')
hold on
    plot(ServoData.PresentSpeed_rad_s_ .* -1, 'Color', blue);
    plot(present_speed, '-.', 'Color', orange)
    legend("Real", "Sim", 'Location', 'southeast');
    ylabel("Speed (rad/s)");
    title("");
    axis([0 300 -5 5]);
hold off

ax3 = subplot(3,1,3);
grid('on')
hold on
    plot(ServoData.Current_mA_, 'Color', blue);
    plot(current(1:10000:end) .* 1e3, '-.', 'Color', orange);
    legend("Real", "Sim", 'Location', 'southeast');
    ylabel("Current (mA)");
    xlabel("Sample");
    axis([0 300 -0.5 0.5]);
hold off

linkprop([ax1,ax2,ax3],{'GridLineStyle'}); 
linkprop([ax1,ax2],{'xlabel'});
linkprop([ax2,ax3],{'title'});