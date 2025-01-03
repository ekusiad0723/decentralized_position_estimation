% filepath: /c:/Users/daisu/Documents/GitHub/decentralized_position_estimation/kf3Sat2Dimension.m
% 定数の定義
SIMULATION_TIME = 5400;  % simulation time
Q = 0;        % process noise variance
R = 0;       % measurement noise variance
L = 0.3;       % target distance
MASS = 0.5;         % mass of the satellite
KP = 0.000001; % P gain
KD = 0.002;  % D gain
SATURATION_LIMIT = 0.05e-6; % 飽和入力の制限 N

FRAME_RATE = 30; % frame rate of the video
DT_ERROR = 0.1;  %誤差更新時間
TIMES_SPEED = 600; %動画の時間の速度倍率
SPACE_SIZE = [-0.2, 0.2]; % 衛星の初期位置の範囲
SIMULATION_VIDEO_WINDOW_POSITION = [0, 100, 1280, 720]; % ウィンドウの位置　[left bottom width height]
FORCE_PLOT_WINDOW_POSITION = [100, 200, 1280, 720]; % ウィンドウの位置　[left bottom width height]
disp("calculating satellite trajectory...");

% 衛星の初期位置
xr1 = [-0.2 + 0.4 * rand; -0.2 + 0.4 * rand];
xr2 = [-0.2 + 0.4 * rand; -0.2 + 0.4 * rand];
xr3 = [-0.2 + 0.4 * rand; -0.2 + 0.4 * rand];

%誤差を更新しながら、PD制御で衛星間の距離をLに制御
stepNum = ceil(SIMULATION_TIME/DT_ERROR);
% 誤差を先に生成
errors = randn(9, stepNum);

% 衛星の初期位置 状態変数z = [x1, y1, x2, y2, x3, y3, vx1, vy1, vx2, vy2, vx3, vy3]
z0 = [xr1(1), xr1(2), xr2(1), xr2(2), xr3(1), xr3(2), 0, 0, 0, 0, 0, 0];

options = odeset("MaxStep", TIMES_SPEED/FRAME_RATE/2); % 最大ステップ幅を設定 アニメーションがカクカクにならないように
[t, z] = ode45(@(t, z) odeFcn(t, z, KP, KD, MASS, L, SATURATION_LIMIT), [0, SIMULATION_TIME], z0,  options);

disp("satellite trajectory calculated.");
disp("creating video...");

% 動画ファイルの設定
if ~exist('result', 'dir')
    mkdir('result'); % resultフォルダが存在しない場合は作成
end
if ~exist(fullfile('result', 'trajectoryVideo'), 'dir')
    mkdir(fullfile('result', 'trajectoryVideo')); % trajectoryVideoフォルダが存在しない場合は作成
end
if ~exist(fullfile('result', 'forceplot'), 'dir')
    mkdir(fullfile('result', 'forceplot')); % forceplotフォルダが存在しない場合は作成
end

% ファイル名の生成
dateStr = datetime("now", "Format", "yyyyMMdd");
fileIndex = 1;

while exist(fullfile('result', 'trajectoryVideo', sprintf('%s_%d_satelliteTrajectory.mp4', dateStr, fileIndex)), 'file') || exist(fullfile('result', 'forceplot', sprintf('%s_%d_forcePlot.png', dateStr, fileIndex)), 'file')
    fileIndex = fileIndex + 1;
end
videoFile = fullfile('result', 'trajectoryVideo', sprintf('%s_%d_satelliteTrajectory.mp4', dateStr, fileIndex));

% simulationVideoWriterクラスを使用して動画を書き出す
videoWriter = simulationVideoWriter(FRAME_RATE, TIMES_SPEED, SIMULATION_TIME, SPACE_SIZE, SIMULATION_VIDEO_WINDOW_POSITION, videoFile);

% 初期位置を追加
videoWriter.addStaticObject(z(1, 1), z(1, 2), 'r', 'x'); % 衛星1の初期位置
videoWriter.addStaticObject(z(1, 3), z(1, 4), 'g', 'x'); % 衛星2の初期位置
videoWriter.addStaticObject(z(1, 5), z(1, 6), 'b', 'x'); % 衛星3の初期位置

% 動的オブジェクトを追加
videoWriter.addDynamicObject(z(:, 1), z(:, 2), 'r', 'o'); % 衛星1の現在位置
videoWriter.addDynamicObject(z(:, 3), z(:, 4), 'g', 'o'); % 衛星2の現在位置
videoWriter.addDynamicObject(z(:, 5), z(:, 6), 'b', 'o'); % 衛星3の現在位置

videoWriter.writeVideo(t);

disp("video created.");

% 各衛星の出力の大きさを計算
dt = diff(t);
F1_x = MASS * diff(z(:, 7)) ./ dt * 1e6; % N -> μN
F1_y = MASS * diff(z(:, 8)) ./ dt * 1e6; % N -> μN
F2_x = MASS * diff(z(:, 9)) ./ dt * 1e6; % N -> μN
F2_y = MASS * diff(z(:, 10)) ./ dt * 1e6; % N -> μN
F3_x = MASS * diff(z(:, 11)) ./ dt * 1e6; % N -> μN
F3_y = MASS * diff(z(:, 12)) ./ dt * 1e6; % N -> μN

% 出力の大きさをプロット
figure('Name', 'Force Plot', 'Position', FORCE_PLOT_WINDOW_POSITION);
plot(t(1:end-1), F1_x, 'r', 'DisplayName', 'Satellite 1 X axis');
hold on;
plot(t(1:end-1), F1_y, 'r--', 'DisplayName', 'Satellite 1 Y axis');
plot(t(1:end-1), F2_x, 'g', 'DisplayName', 'Satellite 2 X axis');
plot(t(1:end-1), F2_y, 'g--', 'DisplayName', 'Satellite 2 Y axis');
plot(t(1:end-1), F3_x, 'b', 'DisplayName', 'Satellite 3 X axis');
plot(t(1:end-1), F3_y, 'b--', 'DisplayName', 'Satellite 3 Y axis');
xlabel('Time (s)');
ylabel('Force (μN)');
title('Output Force of Satellites');
legend;
hold off;

% force plotの保存
forcePlotFile = fullfile('result', 'forceplot', sprintf('%s_%d_forcePlot.png', dateStr, fileIndex));
saveas(gcf, forcePlotFile);

disp("force plot created.");

% 制御力のみをフィードバック
function dydt = odeFcn(t, y, KP, KD, MASS, L, SATURATION_LIMIT)
    dydt = zeros(12, 1);
    % 衛星の状態量
    dydt(1) = y(7);
    dydt(2) = y(8);
    dydt(3) = y(9);
    dydt(4) = y(10);
    dydt(5) = y(11);
    dydt(6) = y(12);
    % 衛星間の距離
    r12 = sqrt((y(1)-y(3))^2 + (y(2)-y(4))^2);
    r23 = sqrt((y(3)-y(5))^2 + (y(4)-y(6))^2);
    r31 = sqrt((y(5)-y(1))^2 + (y(6)-y(2))^2);
    % 加速度フィードバック制御
    u1 = (1/MASS)*(-KP*((L-r12)/r12*(y(3)-y(1)) + (L-r31)/r31*(y(5)-y(1))) - KD*y(7));
    u2 = (1/MASS)*(-KP*((L-r12)/r12*(y(4)-y(2)) + (L-r31)/r31*(y(6)-y(2))) - KD*y(8));
    u3 = (1/MASS)*(-KP*((L-r12)/r12*(y(1)-y(3)) + (L-r23)/r23*(y(5)-y(3))) - KD*y(9));
    u4 = (1/MASS)*(-KP*((L-r12)/r12*(y(2)-y(4)) + (L-r23)/r23*(y(6)-y(4))) - KD*y(10));
    u5 = (1/MASS)*(-KP*((L-r23)/r23*(y(3)-y(5)) + (L-r31)/r31*(y(1)-y(5))) - KD*y(11));
    u6 = (1/MASS)*(-KP*((L-r23)/r23*(y(4)-y(6)) + (L-r31)/r31*(y(2)-y(6))) - KD*y(12));
    
    % 飽和入力の適用
    dydt(7) = max(min(u1, SATURATION_LIMIT/MASS), -SATURATION_LIMIT/MASS);
    dydt(8) = max(min(u2, SATURATION_LIMIT/MASS), -SATURATION_LIMIT/MASS);
    dydt(9) = max(min(u3, SATURATION_LIMIT/MASS), -SATURATION_LIMIT/MASS);
    dydt(10) = max(min(u4, SATURATION_LIMIT/MASS), -SATURATION_LIMIT/MASS);
    dydt(11) = max(min(u5, SATURATION_LIMIT/MASS), -SATURATION_LIMIT/MASS);
    dydt(12) = max(min(u6, SATURATION_LIMIT/MASS), -SATURATION_LIMIT/MASS);
end
