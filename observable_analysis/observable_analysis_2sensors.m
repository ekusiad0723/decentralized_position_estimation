% filepath: /c:/Users/daisu/Documents/GitHub/decentralized_position_estimation/observable_analysis/observable_analysis_2sensors.m
%% Constants
SENSOR_FOV = 90;        % degrees
SNSOR_RANGE = 1;         % meters
COMMUNICATION_RANGE = 3; % meters
N = 1000;
SPACE_DIAMETER = 10;
PLOT_RANGE = SPACE_DIAMETER/2 + 1;

%% Generate random points and satellite attitudes
theta = 2*pi*rand(N,1);
r = (SPACE_DIAMETER/2)*sqrt(rand(N,1));
points = [r.*cos(theta), r.*sin(theta)];

attitudes = 360 * rand(N, 1);  % satellite attitude angles in degrees

%% Generate adjacency matrices for two sensors (front and back)
sensorObservableMatrix_front = zeros(N);
sensorObservableMatrix_back  = zeros(N);
communicationMatrix = zeros(N);

for i = 1:N
    % Front sensor: pointing in the direction of satellite's attitude
    sensorAngle_front = deg2rad(attitudes(i));
    % Back sensor: opposite方向（角度にπを加算）
    sensorAngle_back = wrapAngle(sensorAngle_front + pi);
    
    for j = 1:N
        if i == j
            continue;  % Skip self-observation
        end
        
        dx = points(j,1) - points(i,1);
        dy = points(j,2) - points(i,2);
        distance = sqrt(dx^2 + dy^2);
        
        % For sensor observation, the target satellite must be within sensor range
        if distance <= SNSOR_RANGE
            targetAngle = atan2(dy, dx);
            % Front sensor check
            angleDiff_front = wrapAngle(targetAngle - sensorAngle_front);
            if abs(angleDiff_front) <= deg2rad(SENSOR_FOV/2)
                sensorObservableMatrix_front(i,j) = 1;
            end
            % Back sensor check
            angleDiff_back = wrapAngle(targetAngle - sensorAngle_back);
            if abs(angleDiff_back) <= deg2rad(SENSOR_FOV/2)
                sensorObservableMatrix_back(i,j) = 1;
            end
        end
    end
end

%% Communication matrix (same as before)
for i = 1:N
    for j = 1:N
        if i == j
            continue;
        end
        dx = points(j,1) - points(i,1);
        dy = points(j,2) - points(i,2);
        distance = sqrt(dx^2 + dy^2);
        if distance <= COMMUNICATION_RANGE
            communicationMatrix(i,j) = 1;
            communicationMatrix(j,i) = 1;
        end
    end
end

%% Combine both sensor matrices (logical OR)
sensorObservableMatrix_total = sensorObservableMatrix_front | sensorObservableMatrix_back;
visionObservableMatrix = sensorObservableMatrix_total & communicationMatrix;
lidarObservableMatrix  = sensorObservableMatrix_total & sensorObservableMatrix_total' & communicationMatrix;

%% Prepare save folder and filename constant string
saveFolder = fullfile('result', 'observable_analysis', '2sensors');
if ~exist(saveFolder, 'dir')
    mkdir(saveFolder);
end
constStr = sprintf('FOV%d_RANGE%d_N%d_D%d_2sensors', SENSOR_FOV, SNSOR_RANGE, N, SPACE_DIAMETER);

%% Plot and save Sensor Observable Graph (Union of two sensors)
fig1 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
p1 = plot(digraph(sensorObservableMatrix_total), 'XData', points(:,1), 'YData', points(:,2), ...
    'EdgeAlpha', 0.5, 'LineWidth',1.5, 'ArrowSize',12);
title('Sensor Observable Graph (Front+Back)');
axis equal;
saveFigureFHD(fig1, ['SensorObservableGraph_' constStr], saveFolder);

%% Plot and save Communication Graph
fig2 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
p2 = plot(graph(communicationMatrix), 'XData', points(:,1), 'YData', points(:,2));
title('Communication Graph');
axis equal;
saveFigureFHD(fig2, ['CommunicationGraph_' constStr], saveFolder);

%% Plot and save Vision Observable Graph
fig3 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
p3 = plot(digraph(visionObservableMatrix), 'XData', points(:,1), 'YData', points(:,2), ...
    'EdgeAlpha',0.5, 'LineWidth',1.5, 'ArrowSize',12);
title('Vision Observable Graph (Union)');
axis equal;
saveFigureFHD(fig3, ['VisionObservableGraph_' constStr], saveFolder);

%% Plot and save Lidar Observable Graph
fig4 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
p4 = plot(graph(lidarObservableMatrix), 'XData', points(:,1), 'YData', points(:,2));
title('Lidar Observable Graph (Union)');
axis equal;
saveFigureFHD(fig4, ['LidarObservableGraph_' constStr], saveFolder);

%% Plot and save Sensor Field-of-View for Each Satellite (showing both sensor arcs)
fig5 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
hold on;
axis equal;
scatter(points(:,1), points(:,2), 25, 'b', 'filled');
arcPoints = 20;
for k = 1:N
    x0 = points(k,1);
    y0 = points(k,2);
    % Front sensor arc
    front_angle = deg2rad(attitudes(k));
    theta1 = front_angle - deg2rad(SENSOR_FOV/2);
    theta2 = front_angle + deg2rad(SENSOR_FOV/2);
    theta_front = linspace(theta1, theta2, arcPoints);
    arcX_front = x0 + SNSOR_RANGE*cos(theta_front);
    arcY_front = y0 + SNSOR_RANGE*sin(theta_front);
    patchX_front = [x0, arcX_front, x0];
    patchY_front = [y0, arcY_front, y0];
    patch(patchX_front, patchY_front, 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    
    % Back sensor arc
    back_angle = wrapAngle(front_angle + pi);
    theta1_back = back_angle - deg2rad(SENSOR_FOV/2);
    theta2_back = back_angle + deg2rad(SENSOR_FOV/2);
    theta_back = linspace(theta1_back, theta2_back, arcPoints);
    arcX_back = x0 + SNSOR_RANGE*cos(theta_back);
    arcY_back = y0 + SNSOR_RANGE*sin(theta_back);
    patchX_back = [x0, arcX_back, x0];
    patchY_back = [y0, arcY_back, y0];
    patch(patchX_back, patchY_back, 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end
xlabel('X (m)');
ylabel('Y (m)');
title('Sensor Field-of-View for Each Satellite (Front: red, Back: green)');
saveFigureFHD(fig5, ['SensorFieldOfView_' constStr], saveFolder);

%% Local function: wrapAngle
function angleWrapped = wrapAngle(angle)
    % wrapAngle - Wrap angle to the interval [-pi, pi]
    angleWrapped = mod(angle + pi, 2*pi) - pi;
end