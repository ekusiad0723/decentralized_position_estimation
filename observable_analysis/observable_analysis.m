% Constants
SENSOR_FOV = 120; % degrees
SNSOR_RANGE = 1; % meters
COMMUNICATION_RANGE = 3; % meters
N = 1000;
SPACE_DIAMETER = 10;

PLOT_RANGE = SPACE_DIAMETER/2 + 1;


% Generate random points
theta = 2*pi*rand(N,1);
r = (SPACE_DIAMETER/2)*sqrt(rand(N,1));
points = [r.*cos(theta), r.*sin(theta)];

attitudes = 360 * rand(N, 1);  % Generate a random column vector of N satellite attitude angles in degrees


% Generate adjacency matrix
sensorObservableMatrix = zeros(N);
communicationMatrix = zeros(N);

for i = 1:N
    
    sensorAngle = deg2rad(attitudes(i));  % Convert satellite i's attitude to radians
    
    for j = 1:N
        if i == j
            continue;  % Skip self-observation
        end
        
        dx = points(j,1) - points(i,1);
        dy = points(j,2) - points(i,2);
        distance = sqrt(dx^2 + dy^2);
        
        % Check if satellite j is within sensor range of satellite i
        if distance <= SNSOR_RANGE
            % Calculate the angle from satellite i to satellite j
            targetAngle = atan2(dy, dx);
            % Compute the minimal angular difference, wrapped between -pi and pi
            angleDiff = targetAngle - sensorAngle;
            if abs(angleDiff) <= deg2rad(SENSOR_FOV/2)
                sensorObservableMatrix(i,j) = 1;
            end
        end
    end
end

for i = 1:N
    
    for j = 1:N
        if i == j
            continue;  % Skip self-observation
        end
        
        dx = points(j,1) - points(i,1);
        dy = points(j,2) - points(i,2);
        distance = sqrt(dx^2 + dy^2);
        
        % Check if satellite j is within communication range of satellite i
        if distance <= COMMUNICATION_RANGE
            communicationMatrix(i,j) = 1;
            communicationMatrix(j,i) = 1;
        end
    end
end

visionObservableMatrix = sensorObservableMatrix & communicationMatrix;
lidarObservableMatrix  = sensorObservableMatrix & sensorObservableMatrix' & communicationMatrix;

% 保存先フォルダの準備
saveFolder = fullfile('result', 'observable_analysis', '1sensor');
if ~exist(saveFolder, 'dir')
    mkdir(saveFolder);
end

% 定数情報をファイル名に埋め込む文字列例
constStr = sprintf('FOV%d_RANGE%d_N%d_D%d', SENSOR_FOV, SNSOR_RANGE, N, SPACE_DIAMETER);

%% Sensor Observable Graph
fig1 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
p1 = plot(digraph(sensorObservableMatrix), 'XData', points(:,1), 'YData', points(:,2), ...
    'EdgeAlpha', 0.5, 'LineWidth', 1.5, 'ArrowSize', 12);
title('Sensor Observable Graph');
axis equal;
saveFigureFHD(fig1, ['SensorObservableGraph_' constStr], saveFolder);

%% Communication Graph
fig2 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
p2 = plot(graph(communicationMatrix), 'XData', points(:,1), 'YData', points(:,2));
title('Communication Graph');
axis equal;
saveFigureFHD(fig2, ['CommunicationGraph_' constStr], saveFolder);

%% Vision Observable Graph
fig3 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
p3 = plot(digraph(sensorObservableMatrix & communicationMatrix), 'XData', points(:,1), 'YData', points(:,2), ...
    'EdgeAlpha', 0.5, 'LineWidth', 1.5, 'ArrowSize', 12);
title('Vision Observable Graph');
axis equal;
saveFigureFHD(fig3, ['VisionObservableGraph_' constStr], saveFolder);

%% Lidar Observable Graph
fig4 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
p4 = plot(graph(sensorObservableMatrix & sensorObservableMatrix' & communicationMatrix), 'XData', points(:,1), 'YData', points(:,2));
title('Lidar Observable Graph');
axis equal;
saveFigureFHD(fig4, ['LidarObservableGraph_' constStr], saveFolder);

%% Sensor Field-of-View for Each Satellite
fig5 = figure('Units','normalized','OuterPosition',[0 0 1 1]);
hold on;
axis equal;
scatter(points(:,1), points(:,2), 25, 'b', 'filled');
arcPoints = 20;
for k = 1:N
    x0 = points(k,1);
    y0 = points(k,2);
    att_rad = deg2rad(attitudes(k));
    
    theta1 = att_rad - deg2rad(SENSOR_FOV/2);
    theta2 = att_rad + deg2rad(SENSOR_FOV/2);
    theta = linspace(theta1, theta2, arcPoints);
    
    arcX = x0 + SNSOR_RANGE * cos(theta);
    arcY = y0 + SNSOR_RANGE * sin(theta);
    
    patchX = [x0, arcX, x0];
    patchY = [y0, arcY, y0];
    patch(patchX, patchY, 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end
xlabel('X (m)');
ylabel('Y (m)');
title('Sensor Field-of-View for Each Satellite');
saveFigureFHD(fig5, ['SensorFieldOfView_' constStr], saveFolder);

