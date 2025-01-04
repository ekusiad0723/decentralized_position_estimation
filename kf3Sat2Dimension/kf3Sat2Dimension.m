% filepath: /c:/Users/daisu/Documents/GitHub/decentralized_position_estimation/kf3Sat2Dimension.m
% 定数の定義
SIMULATION_TIME = 10800;  % simulation time
Q = 0;        % process noise variance
R = 0;       % measurement noise variance
L = 0.3;       % target distance
M = 0.5;         % mass of the satellite
KP = 0.000001; % P gain
KD =  0.0005;  % D gain
SATURATION_LIMIT =  0.05e-6; % 飽和入力の制限 N

FRAME_RATE = 30; % frame rate of the video
TIME_STEP = 0.1; % time step
TIMES_SPEED = 600; %動画の時間の速度倍率
SPACE_SIZE = [-0.4, 0.4]; % 衛星の初期位置の範囲
SIMULATION_VIDEO_WINDOW_POSITION = [0, 100, 1280, 720]; % ウィンドウの位置　[left bottom width height]
FORCE_PLOT_WINDOW_POSITION = [100, 200, 1280, 720]; % ウィンドウの位置　[left bottom width height]
disp("calculating satellite trajectory...");

% 衛星の初期位置
xr1 = [-0.2 + 0.4 * rand; -0.2 + 0.4 * rand];
xr2 = [-0.2 + 0.4 * rand; -0.2 + 0.4 * rand];
xr3 = [-0.2 + 0.4 * rand; -0.2 + 0.4 * rand];

% 衛星の初期位置 状態変数z = [x1, y1, x2, y2, x3, y3, vx1, vy1, vx2, vy2, vx3, vy3]
z0 = [xr1(1), xr1(2), xr2(1), xr2(2), xr3(1), xr3(2), 0, 0, 0, 0, 0, 0];

options = odeset("MaxStep", TIMES_SPEED/FRAME_RATE/2); % 最大ステップ幅を設定 アニメーションがカクカクにならないように
[t, z] = ode45(@(t, z) odeFcn(t, z, KP, KD, M, L, SATURATION_LIMIT), [0, SIMULATION_TIME], z0,  options);
disp("satellite trajectory calculated.");


% Position estimation using Kalmanfilter
% Create equally spaced time points
tDiscrete = (0:TIME_STEP:max(t))';
% Interpolate state
zDiscrete = interp1(t, z, tDiscrete);
stepNum = ceil(SIMULATION_TIME/TIME_STEP);
% 誤差を先に生成
errors = randn(9, stepNum);



disp("creating video...");

% 動画ファイルの設定
if ~exist('result', 'dir')
    mkdir('result'); % resultフォルダが存在しない場合は作成
end

% kf3Sat2Dimensionフォルダの作成
if ~exist(fullfile('result','kf3Sat2Dimension'),'dir')
    mkdir(fullfile('result','kf3Sat2Dimension'));
end

% 日付フォルダの作成
dateStr = string(datetime("now", "Format", "yyyyMMdd"));  % string に変換
baseDir = fullfile('result','kf3Sat2Dimension', dateStr);
if ~exist(baseDir,'dir')
    mkdir(baseDir);
end

fileIndex = 1;
% 動画とforcePlotの両方を同じフォルダに保存するための重複チェック
while exist(fullfile(baseDir, sprintf('%s_%d_satelliteTrajectoryInertialFrame.mp4', dateStr, fileIndex)), 'file') ...
      || exist(fullfile(baseDir, sprintf('%s_%d_forcePlot.png', dateStr, fileIndex)), 'file')
    fileIndex = fileIndex + 1;
end

videoFile = fullfile(baseDir, sprintf('%s_%d_satelliteTrajectory.mp4', dateStr, fileIndex));

% simulationVideoWriterクラスを使用して動画を書き出す
inertialFrameSimulationVideo = simulationVideoWriter(FRAME_RATE, TIMES_SPEED, SIMULATION_TIME, SPACE_SIZE, 'Simulation in Inertial Frame', SIMULATION_VIDEO_WINDOW_POSITION, videoFile);
inertialFrameSimulationVideo.addTime(t); % 時間を追加

% 初期位置を追加
inertialFrameSimulationVideo.addStaticObject(z(1, 1), z(1, 2), 'r', 'x'); % 衛星1の初期位置
inertialFrameSimulationVideo.addStaticObject(z(1, 3), z(1, 4), 'g', 'x'); % 衛星2の初期位置
inertialFrameSimulationVideo.addStaticObject(z(1, 5), z(1, 6), 'b', 'x'); % 衛星3の初期位置

% 動的オブジェクトを追加
inertialFrameSimulationVideo.addDynamicObject(z(:, 1), z(:, 2), 'r', 'o'); % 衛星1の現在位置
inertialFrameSimulationVideo.addDynamicObject(z(:, 3), z(:, 4), 'g', 'o'); % 衛星2の現在位置
inertialFrameSimulationVideo.addDynamicObject(z(:, 5), z(:, 6), 'b', 'o'); % 衛星3の現在位置

inertialFrameSimulationVideo.writeVideo();

disp("video created.");

% 各衛星の出力の大きさを計算
dt = diff(t);
F1_x = M * diff(z(:, 7)) ./ dt * 1e6; % N -> μN
F1_y = M * diff(z(:, 8)) ./ dt * 1e6; % N -> μN
F2_x = M * diff(z(:, 9)) ./ dt * 1e6; % N -> μN
F2_y = M * diff(z(:, 10)) ./ dt * 1e6; % N -> μN
F3_x = M * diff(z(:, 11)) ./ dt * 1e6; % N -> μN
F3_y = M * diff(z(:, 12)) ./ dt * 1e6; % N -> μN

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
forcePlotFile = fullfile(baseDir, sprintf('%s_%d_forcePlot.png', dateStr, fileIndex));
saveas(gcf, forcePlotFile);

disp("force plot created.");

% 制御力のみをフィードバック
function dydt = odeFcn(t, y, KP, KD, M, L, SATURATION_LIMIT)
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

    %偏差
    e1 =  ((L-r12)*(y(1)-y(3))/r12 + (L-r31)*(y(1)-y(5))/r31);
    e2 =  ((L-r12)*(y(2)-y(4))/r12 + (L-r31)*(y(2)-y(6))/r31);
    e3 =  ((L-r23)*(y(3)-y(5))/r23 + (L-r12)*(y(3)-y(1))/r12);
    e4 =  ((L-r23)*(y(4)-y(6))/r23 + (L-r12)*(y(4)-y(2))/r12);
    e5 =  ((L-r31)*(y(5)-y(1))/r31 + (L-r23)*(y(5)-y(3))/r23);
    e6 =  ((L-r31)*(y(6)-y(2))/r31 + (L-r23)*(y(6)-y(4))/r23); 
    
    % 衛星間の距離の微分
    dr12 = ((y(3)-y(1))*(y(9)-y(7)) + (y(4)-y(2))*(y(10)-y(8))) / r12;
    dr23 = ((y(5)-y(3))*(y(11)-y(9)) + (y(6)-y(4))*(y(12)-y(10))) / r23;
    dr31 = ((y(1)-y(5))*(y(7)-y(11)) + (y(2)-y(6))*(y(8)-y(12))) / r31;

    % 偏差の微分（ノルムだけ微分すればいいことに気づき！！）
    de1 = -dr12 * (y(1)-y(3)) / r12 -dr31 * (y(1)-y(5)) / r31;
    de2 = -dr12 * (y(2)-y(4)) / r12 -dr31 * (y(2)-y(6)) / r31;
    de3 = -dr23 * (y(3)-y(5)) / r23 -dr12 * (y(3)-y(1)) / r12;
    de4 = -dr23 * (y(4)-y(6)) / r23 -dr12 * (y(4)-y(2)) / r12;
    de5 = -dr31 * (y(5)-y(1)) / r31 -dr23 * (y(5)-y(3)) / r23;
    de6 = -dr31 * (y(6)-y(2)) / r31 -dr23 * (y(6)-y(4)) / r23;
    
    % PD制御
    u1 = (1/M)*(KP*e1 + KD*de1);
    u2 = (1/M)*(KP*e2 + KD*de2);
    u3 = (1/M)*(KP*e3 + KD*de3);
    u4 = (1/M)*(KP*e4 + KD*de4);
    u5 = (1/M)*(KP*e5 + KD*de5);
    u6 = (1/M)*(KP*e6 + KD*de6);
    
    % 飽和入力を適用して、saturation-PD制御
    dydt(7) = max(min(u1, SATURATION_LIMIT/M), -SATURATION_LIMIT/M);
    dydt(8) = max(min(u2, SATURATION_LIMIT/M), -SATURATION_LIMIT/M);
    dydt(9) = max(min(u3, SATURATION_LIMIT/M), -SATURATION_LIMIT/M);
    dydt(10) = max(min(u4, SATURATION_LIMIT/M), -SATURATION_LIMIT/M);
    dydt(11) = max(min(u5, SATURATION_LIMIT/M), -SATURATION_LIMIT/M);
    dydt(12) = max(min(u6, SATURATION_LIMIT/M), -SATURATION_LIMIT/M);
end
