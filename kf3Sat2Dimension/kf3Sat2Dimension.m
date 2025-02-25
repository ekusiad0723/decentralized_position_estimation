% filepath: /c:/Users/daisu/Documents/GitHub/decentralized_position_estimation/kf3Sat2Dimension.m
% 定数の定義
SIMULATION_TIME = 10800;  % simulation time
L = 0.3;       % target distance
M = 0.5;         % mass of the satellite
KP = 0.000001; % P gain
KD =  0.001;  % D gain
SATURATION_LIMIT =  0.05e-6; % 飽和入力の制限 N

FRAME_RATE = 30; % frame rate of the video
TIMES_SPEED = 1800; %動画の時間の速度倍率
SPACE_SIZE = [-0.4, 0.4]; % 衛星の初期位置の範囲
INERTIAL_FRAME_SIMULATION_VIDEO_WINDOW_POSITION = [0, 100, 1280, 720]; % ウィンドウの位置　[left bottom width height]
SAT1_FRAME_SIMULATION_VIDEO_WINDOW_POSITION = [100, 150, 1280, 720]; % ウィンドウの位置　[left bottom width height]
ROTATED_SAT1_FRAME_SIMULATION_VIDEO_WINDOW_POSITION = [200, 200, 1280, 720]; % ウィンドウの位置　[left bottom width height]
FORCE_PLOT_WINDOW_POSITION = [200, 200, 1280, 720]; % ウィンドウの位置　[left bottom width height]

% 時系列モデル
% z(k+1) = A * z(k) + b * v(k) + u(k)
% r(k) = h(z(k)) + w(k)
TIME_STEP = 0.1; % time step　[s]　ukfStateFcnの中の定義と一致させる
Q = 0.0001;        % process noise variance
R = 0.0001;       % measurement noise variance


% 衛星の初期位置
xr1 = [-0.2+0.4*rand(), -0.2+0.4*rand()];
xr2 = [-0.2+0.4*rand(), -0.2+0.4*rand()];
xr3 = [-0.2+0.4*rand(), -0.2+0.4*rand()];

disp("calculating satellite trajectory...");
% 衛星の初期位置 状態変数z = [x1, y1, x2, y2, x3, y3, vx1, vy1, vx2, vy2, vx3, vy3]
z0 = [xr1(1), xr1(2), xr2(1), xr2(2), xr3(1), xr3(2), 0, 0, 0, 0, 0, 0];

options = odeset("MaxStep", TIME_STEP/2); %ODEの仕様か分からないが、なぜかこのシミュレーションではステップを小さくしないときれいな計算がされない
[t, z] = ode45(@(t, z) odeFcn(t, z, KP, KD, M, L, SATURATION_LIMIT), [0, SIMULATION_TIME], z0, options);

% Position estimation using Kalmanfilter
% Create equally spaced time points
tDiscrete = (0:TIME_STEP:max(t))';
% Interpolate state
zDiscrete = interp1(t, z, tDiscrete);
stepNum = ceil(SIMULATION_TIME/TIME_STEP);

zSat1Frame = [
    zDiscrete(:,3)  - zDiscrete(:,1), zDiscrete(:,4)  - zDiscrete(:,2), zDiscrete(:,5)  - zDiscrete(:,1),  zDiscrete(:,6)  - zDiscrete(:,2), ...
    zDiscrete(:,9)  - zDiscrete(:,7), zDiscrete(:,10) - zDiscrete(:,8), zDiscrete(:,11) - zDiscrete(:,7), zDiscrete(:,12) - zDiscrete(:,8)];


zSat1FrameRotated = zeros(size(zSat1Frame, 1), 6);
for i = 1:size(zSat1Frame,1)
    % 0 <= ang < 2pi
    if zSat1Frame(1) >= 0 && zSat1Frame(2) >= 0 
        ang = atan(zSat1Frame(i,2)/zSat1Frame(i,1));
    elseif zSat1Frame(1) < 0 && zSat1Frame(2) >= 0
        ang = pi + atan(zSat1Frame(i,2)/zSat1Frame(i,1));
    elseif zSat1Frame(1) < 0 && zSat1Frame(2) < 0
        ang = pi + atan(zSat1Frame(i,2)/zSat1Frame(i,1));
    else
        ang = 2*pi + atan(zSat1Frame(i,2)/zSat1Frame(i,1));
    end
    rotater = [cos(-ang), -sin(-ang); sin(-ang), cos(-ang)];
    r12 = sqrt(zSat1Frame(i,1)^2 + zSat1Frame(i,2)^2);
    zSat1FrameRotated(i,1)      = r12;
    zSat1FrameRotated(i,2:3)    = (rotater * zSat1Frame(i,3:4)')';
    zSat1FrameRotated(i,4)      = (zSat1Frame(i,1)*zSat1Frame(i,5)+zSat1Frame(i,2)*zSat1Frame(i,6))/r12;
    zSat1FrameRotated(i,5)      = (zSat1Frame(i,1)*zSat1Frame(i,7)-zSat1Frame(i,2)*zSat1Frame(i,8))/r12 ...
                                + (zSat1Frame(i,1)*zSat1Frame(i,6)-zSat1Frame(i,5)*zSat1Frame(i,2))*(-zSat1Frame(i,2)*zSat1Frame(i,3)-zSat1Frame(i,1)*zSat1Frame(i,4))/r12^3;
    zSat1FrameRotated(i,6)      = -(zSat1Frame(i,1)*zSat1Frame(i,8)+zSat1Frame(i,2)*zSat1Frame(i,7))/r12 ...
                                - (zSat1Frame(i,1)*zSat1Frame(i,6)-zSat1Frame(i,5)*zSat1Frame(i,2))*(zSat1Frame(i,1)*zSat1Frame(i,3)-zSat1Frame(i,2)*zSat1Frame(i,4))/r12^3;
end

% disp(zSat1Frame);
%u：制御入力
u = [zeros(1,6);...
    zeros(stepNum,3), diff(zSat1FrameRotated(:,4:6))
]; 
Pz0 =  0.5 * eye(6); %共分散行列の初期値、SNRが大きい程係数を小さく設定する
zHat0 = [zSat1FrameRotated(1,1:3),zeros(1,3)];  %状態推定値の初期値
%zHat0 = zeros(1,6);
zHat = [zHat0; zeros(stepNum, 6)];
obj = unscentedKalmanFilter(@ukfStateFcn, @ukfMeasurementFcn, zHat0);
obj.StateCovariance = Pz0;
obj.ProcessNoise = Q*[zeros(3), zeros(3); zeros(3),TIME_STEP/M*eye(3)];
obj.MeasurementNoise = R;
for k = 1:stepNum
    obj.predict(u(k,:)');
    obj.correct([sqrt((zDiscrete(k,1)-zDiscrete(k,3))^2 + (zDiscrete(k,2)-zDiscrete(k,4))^2);...
                sqrt((zDiscrete(k,3)-zDiscrete(k,5))^2 + (zDiscrete(k,4)-zDiscrete(k,6))^2);...
                sqrt((zDiscrete(k,5)-zDiscrete(k,1))^2 + (zDiscrete(k,6)-zDiscrete(k,2))^2)]);
    zHat(k+1,:) = obj.State;
end


disp("satellite trajectory calculated.");
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

inertialFrameSimulationVideoFileName ='satellite_trajectory_in_inertial_frame';
sat1FrameSimulationVideoFileName = 'satellite_trajectory_in_sat1_frame';
rotatedSat1FrameSimulationVideoFileName = 'satellite_trajectory_in_sat1_frame_rotated';
forcePlotFileName = 'satellites_control_force';

fileIndex = 1;
% 動画とforcePlotの両方を同じフォルダに保存するための重複チェック
while exist(fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, inertialFrameSimulationVideoFileName)), 'file') ...
        || exist(fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, sat1FrameSimulationVideoFileName)), 'file') ...
        || exist(fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, rotatedSat1FrameSimulationVideoFileName)), 'file') ...
        || exist(fullfile(baseDir, sprintf('%s_%d_%s.png', dateStr, fileIndex, forcePlotFileName)), 'file')
    fileIndex = fileIndex + 1;
end

% ファイル設定
inertialFrameSimulationVideoFile = fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, inertialFrameSimulationVideoFileName));
sat1FrameSimulationVideoFile = fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, sat1FrameSimulationVideoFileName));
rotatedsat1FrameSimulationVideoFile = fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, rotatedSat1FrameSimulationVideoFileName));
forcePlotFile = fullfile(baseDir, sprintf('%s_%d_%s.png', dateStr, fileIndex, forcePlotFileName));

% 慣性系でのシミュレーション動画作成
% simulationVideoWriterクラスを使用して動画を書き出す
inertialFrameSimulationVideo = simulationVideoWriter(FRAME_RATE, TIMES_SPEED, SIMULATION_TIME, SPACE_SIZE, 'Simulation in Inertial Frame', INERTIAL_FRAME_SIMULATION_VIDEO_WINDOW_POSITION, inertialFrameSimulationVideoFile);
inertialFrameSimulationVideo.addTime(tDiscrete); % 時間を追加

% 初期位置を追加
inertialFrameSimulationVideo.addStaticObject(zDiscrete(1, 1), zDiscrete(1, 2), 'r', 'x', 'Satellite 1 Initial Position'); 
inertialFrameSimulationVideo.addStaticObject(zDiscrete(1, 3), zDiscrete(1, 4), 'g', 'x', 'Satellite 2 Initial Position');
inertialFrameSimulationVideo.addStaticObject(zDiscrete(1, 5), zDiscrete(1, 6), 'b', 'x', 'Satellite 3 Initial Position');

% 動的オブジェクトを追加
inertialFrameSimulationVideo.addDynamicObject(zDiscrete(:, 1), zDiscrete(:, 2), 'r', 'o', '-', 'Satellite 1 Current Position', 'Satellite 1 Trajectory');
inertialFrameSimulationVideo.addDynamicObject(zDiscrete(:, 3), zDiscrete(:, 4), 'g', 'o', '-', 'Satellite 2 Current Position', 'Satellite 2 Trajectory');
inertialFrameSimulationVideo.addDynamicObject(zDiscrete(:, 5), zDiscrete(:, 6), 'b', 'o', '-', 'Satellite 3 Current Position', 'Satellite 3 Trajectory');

inertialFrameSimulationVideo.writeVideo();

% 衛星1系でのシミュレーション動画作成
% simulationVideoWriterクラスを使用して動画を書き出す
sat1FrameSimulationVideo = simulationVideoWriter(FRAME_RATE, TIMES_SPEED, SIMULATION_TIME, SPACE_SIZE, 'Simulation in Satellite 1 Frame', SAT1_FRAME_SIMULATION_VIDEO_WINDOW_POSITION, sat1FrameSimulationVideoFile);
sat1FrameSimulationVideo.addTime(tDiscrete);

sat1FrameSimulationVideo.addStaticObject(0,0, 'r', 'o', 'Satellite 1');
sat1FrameSimulationVideo.addStaticObject(zSat1Frame(1,1), zSat1Frame(1,2), 'g', 'x', 'Satellite 2 Initial Position');
sat1FrameSimulationVideo.addStaticObject(zSat1Frame(1,3), zSat1Frame(1,4), 'b', 'x', 'Satellite 3 Initial Position');

sat1FrameSimulationVideo.addDynamicObject(zSat1Frame(:,1), zSat1Frame(:,2), 'g', 'o', '-', 'Satellite 2 Current Position', 'Satellite 2 Trajectory');
sat1FrameSimulationVideo.addDynamicObject(zSat1Frame(:,3), zSat1Frame(:,4), 'b', 'o', '-', 'Satellite 3 Current Position', 'Satellite 3 Trajectory');

sat1FrameSimulationVideo.writeVideo();

%衛星1系（回転済み）でのシミュレーション動画作成
% simulationVideoWriterクラスを使用して動画を書き出す
rotatedSat1FrameSimulationVideo = simulationVideoWriter(FRAME_RATE, TIMES_SPEED, SIMULATION_TIME, SPACE_SIZE, 'Simulation in Satellite 1 Frame Rotated', ROTATED_SAT1_FRAME_SIMULATION_VIDEO_WINDOW_POSITION, rotatedsat1FrameSimulationVideoFile);
rotatedSat1FrameSimulationVideo.addTime(tDiscrete);

rotatedSat1FrameSimulationVideo.addStaticObject(0,0, 'r', 'o', 'Satellite 1');
rotatedSat1FrameSimulationVideo.addStaticObject(zSat1FrameRotated(1,1), 0, 'g', 'x', 'Satellite 2 Initial Position');
rotatedSat1FrameSimulationVideo.addStaticObject(zSat1FrameRotated(1,2), zSat1FrameRotated(1,3), 'b', 'x', 'Satellite 3 Initial Position');

rotatedSat1FrameSimulationVideo.addDynamicObject(zSat1FrameRotated(:,1), zeros(size(zSat1FrameRotated,1),1) , 'g', 'o', '-', 'Satellite 2 Current Position', 'Satellite 2 Trajectory');
rotatedSat1FrameSimulationVideo.addDynamicObject(zSat1FrameRotated(:,2), zSat1FrameRotated(:,3), 'b', 'o', '-', 'Satellite 3 Current Position', 'Satellite 3 Trajectory');

rotatedSat1FrameSimulationVideo.addDynamicObject(zHat(:,1), zeros(size(zHat,1),1) , 'g', '.', ':', 'Satellite 2 Estimated Position', 'Satellite 2 Estimated Trajectory');
rotatedSat1FrameSimulationVideo.addDynamicObject(zHat(:,2), zHat(:,3), 'b', '.', ':', 'Satellite 3 Estimated Position', 'Satellite 3 Estimated Trajectory');

rotatedSat1FrameSimulationVideo.writeVideo();

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
title('Control Force of Satellites');
legend();
hold off;

% force plotの保存
saveas(gcf, forcePlotFile);
close(gcf);
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

function z = ukfStateFcn(z, u)
    TIME_STEP = 0.1;
    A = [eye(3),TIME_STEP * eye(3);...
        zeros(3,3),eye(3)];
    z = A*z + u;
end

function y = ukfMeasurementFcn(z)
    y = [   z(1);...
            sqrt((z(2)-z(1))^2 + z(3)^2);...
            sqrt(z(2)^2 + z(3)^2)];
end

