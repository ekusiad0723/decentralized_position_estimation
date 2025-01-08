% filepath: /c:/Users/daisu/Documents/GitHub/decentralized_position_estimation/kf3Sat2Dimension.m
% 定数の定義
SIMULATION_TIME = 10800;  % simulation time
L = 0.3;       % target distance
M = 0.5;         % mass of the satellite
KP = 0.000001; % P gain
KD =  0.001;  % D gain
SATURATION_LIMIT =  0.05e-6; % 飽和入力の制限 N

FRAME_RATE = 30; % frame rate of the video
TIMES_SPEED = 600; %動画の時間の速度倍率
SPACE_SIZE = [-0.4, 0.4]; % 衛星の初期位置の範囲
INERTIAL_FRAME_SIMULATION_VIDEO_WINDOW_POSITION = [0, 100, 1280, 720]; % ウィンドウの位置　[left bottom width height]
SAT1_FRAME_SIMULATION_VIDEO_WINDOW_POSITION = [100, 150, 1280, 720]; % ウィンドウの位置　[left bottom width height]
FORCE_PLOT_WINDOW_POSITION = [200, 200, 1280, 720]; % ウィンドウの位置　[left bottom width height]

% 時系列モデル
% z(k+1) = A * z(k) + b * v(k) + u(k)
% r(k) = h(z(k)) + w(k)
TIME_STEP = 0.1; % time step
A = [eye(4),TIME_STEP * eye(4);...
    zeros(4,4),eye(4)];
b = [0; 0; 0; 0; TIME_STEP/M; TIME_STEP/M; TIME_STEP/M; TIME_STEP/M];
h = @(z) [  sqrt(z(1).^2 + z(2).^2);...
            sqrt((z(1)-z(3)).^2 + (z(2)-z(4)).^2);...
            sqrt(z(3).^2 + z(4).^2)];
Q = 0;        % process noise variance
R = 0;       % measurement noise variance


% 衛星の初期位置
xr1 = [-0.1; -0.1];
xr2 = [0.1; 0.1];
xr3 = [0.1; -0.1];

disp("calculating satellite trajectory...");
% 衛星の初期位置 状態変数z = [x1, y1, x2, y2, x3, y3, vx1, vy1, vx2, vy2, vx3, vy3]
z0 = [xr1(1), xr1(2), xr2(1), xr2(2), xr3(1), xr3(2), 0, 0, 0, 0, 0, 0];

options = odeset("MaxStep", TIMES_SPEED/FRAME_RATE/2); % 最大ステップ幅を設定 アニメーションがカクカクにならないように
[t, z] = ode45(@(t, z) odeFcn(t, z, KP, KD, M, L, SATURATION_LIMIT), [0, SIMULATION_TIME], z0,  options);

% Position estimation using Kalmanfilter
% Create equally spaced time points
tDiscrete = (0:TIME_STEP:max(t))';
% Interpolate state
zDiscrete = interp1(t, z, tDiscrete);
stepNum = ceil(SIMULATION_TIME/TIME_STEP);

zKf = [
    zDiscrete(:,3)  - zDiscrete(:,1),  zDiscrete(:,4)  - zDiscrete(:,2), ...
    zDiscrete(:,5)  - zDiscrete(:,1),  zDiscrete(:,6)  - zDiscrete(:,2), ...
    zDiscrete(:,9)  - zDiscrete(:,7),  zDiscrete(:,10) - zDiscrete(:,8), ...
    zDiscrete(:,11) - zDiscrete(:,7),  zDiscrete(:,12) - zDiscrete(:,8)
];

%u：制御入力
u = [zeros(1,8);...
    zeros(stepNum,4), diff(zKf(:,5:8))
]; 
Pz =  0.5 * eye(8); %共分散行列の初期値、SNRが大きい程係数を小さく設定する
zHat0 = [zKf(1,1:4),zeros(1,4)];  %状態推定値の初期値
zHat = [zHat0; zeros(stepNum, 8)];

zTest = [zHat0; zeros(stepNum, 8)];
for i = 1:stepNum
    zTest(i+1,:) = (A * zTest(i,:)')' +  u(i,:);  
end
disp("zKf");
disp(zKf(8000:8100,:));
disp("zTest");
disp(zTest(8000:8100,:));

for i = 1:stepNum
    r = h(zKf(i,:)) + sqrt(R) * randn(3,1);
    [zHatTemp, Pz] = halfEkf(A, b, h, Q, R, r, u(i,:)', zHat(i,:)', Pz);
    zHat(i+1,:) = zHatTemp';
end

disp("zHat");
disp(zHat(1:100,:));

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
forcePlotFileName = 'satellites_control_force';

fileIndex = 1;
% 動画とforcePlotの両方を同じフォルダに保存するための重複チェック
while exist(fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, inertialFrameSimulationVideoFileName)), 'file') ...
        || exist(fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, sat1FrameSimulationVideoFileName)), 'file') ...
        || exist(fullfile(baseDir, sprintf('%s_%d_%s.png', dateStr, fileIndex, forcePlotFileName)), 'file')
    fileIndex = fileIndex + 1;
end

% ファイル設定
inertialFrameSimulationVideoFile = fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, inertialFrameSimulationVideoFileName));
sat1FrameSimulationVideoFile = fullfile(baseDir, sprintf('%s_%d_%s.mp4', dateStr, fileIndex, sat1FrameSimulationVideoFileName));
forcePlotFile = fullfile(baseDir, sprintf('%s_%d_%s.png', dateStr, fileIndex, forcePlotFileName));

% 慣性系でのシミュレーション動画作成
% simulationVideoWriterクラスを使用して動画を書き出す
inertialFrameSimulationVideo = simulationVideoWriter(FRAME_RATE, TIMES_SPEED, SIMULATION_TIME, SPACE_SIZE, 'Simulation in Inertial Frame', INERTIAL_FRAME_SIMULATION_VIDEO_WINDOW_POSITION, inertialFrameSimulationVideoFile);
inertialFrameSimulationVideo.addTime(t); % 時間を追加

% 初期位置を追加
inertialFrameSimulationVideo.addStaticObject(z(1, 1), z(1, 2), 'r', 'x', 'Satellite 1 Initial Position'); 
inertialFrameSimulationVideo.addStaticObject(z(1, 3), z(1, 4), 'g', 'x', 'Satellite 2 Initial Position'); 
inertialFrameSimulationVideo.addStaticObject(z(1, 5), z(1, 6), 'b', 'x', 'Satellite 3 Initial Position'); 

% 動的オブジェクトを追加
inertialFrameSimulationVideo.addDynamicObject(z(:, 1), z(:, 2), 'r', 'o', '-', 'Satellite 1 Current Position', 'Satellite 1 Trajectory'); 
inertialFrameSimulationVideo.addDynamicObject(z(:, 3), z(:, 4), 'g', 'o', '-', 'Satellite 2 Current Position', 'Satellite 2 Trajectory');
inertialFrameSimulationVideo.addDynamicObject(z(:, 5), z(:, 6), 'b', 'o', '-', 'Satellite 3 Current Position', 'Satellite 3 Trajectory'); 

inertialFrameSimulationVideo.writeVideo();

% 衛星1系でのシミュレーション動画作成
% simulationVideoWriterクラスを使用して動画を書き出す
sat1FrameSimulationVideo = simulationVideoWriter(FRAME_RATE, TIMES_SPEED, SIMULATION_TIME, SPACE_SIZE, 'Simulation in Satellite 1 Frame', SAT1_FRAME_SIMULATION_VIDEO_WINDOW_POSITION, sat1FrameSimulationVideoFile);
sat1FrameSimulationVideo.addTime(tDiscrete);

sat1FrameSimulationVideo.addStaticObject(0,0, 'r', 'o', 'Satellite 1');
sat1FrameSimulationVideo.addStaticObject(zKf(1,1), zKf(1,2), 'g', 'x', 'Satellite 2 Initial Position');
sat1FrameSimulationVideo.addStaticObject(zKf(1,3), zKf(1,4), 'b', 'x', 'Satellite 3 Initial Position');

sat1FrameSimulationVideo.addDynamicObject(zKf(:,1), zKf(:,2), 'g', 'o', '-', 'Satellite 2 Current Position', 'Satellite 2 Trajectory');
sat1FrameSimulationVideo.addDynamicObject(zKf(:,3), zKf(:,4), 'b', 'o', '-', 'Satellite 3 Current Position', 'Satellite 3 Trajectory');

% sat1FrameSimulationVideo.addDynamicObject(zHat(:,1), zHat(:,2), 'g', '.', ':', 'Satellite 2 Estimated Position', 'Satellite 2 Estimated Trajectory');
% sat1FrameSimulationVideo.addDynamicObject(zHat(:,3), zHat(:,4), 'b', '.', ':', 'Satellite 3 Estimated Position', 'Satellite 3 Estimated Trajectory');

sat1FrameSimulationVideo.writeVideo();

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

function [zHatNext, PzNext] = halfUkf(A, b, h, Q, R, r, u, zHat, Pz)
    % 時系列モデル
    % z(k+1) = A * z(k) + b * v(k) + u(k)
    % r(k) = h(z(k)) + w(k)
    % 予測ステップ
    zHatMinus = A * zHat + u;
    PzMinus = A * Pz * A' + Q*(b*b');
    disp(PzMinus);
    % 更新ステップ
    [rHatMinus, Prr, Prz] = unscentedTransform(h, zHatMinus, PzMinus);
    g = Prz /(Prr + R*eye(6)); % 観測値は他の衛星の距離なので独立なノイズが乗ると考えられる
    zHatNext = zHatMinus + g * (r - rHatMinus);
    PzNext = PzMinus - g * Prz';
end

function [yMean, Pyy, Pxy] = unscentedTransform(f, xMean, Pxx)
    % yMean = function(xMean)に対するU変換
    mapcols = @(f, x) cell2mat(cellfun(f,mat2cell(x,size(x,1),ones(1,size(x,2))),'UniformOutput',false));
    n = length(xMean);
    kappa = 0;
    w0 = kappa/(n+kappa);
    wi = 1/(2*(n+kappa));
    W = diag([w0;wi*ones(2*n,1)]);
    L = chol(Pxx, "lower");
    X = [xMean'; ones(n,1)*xMean' + sqrt(n+kappa)*L; ones(n,1)*xMean' - sqrt(n+kappa)*L];
    Y = mapcols(f, X')';
    yMean = sum(W*Y)';
    Yd = Y - ones(2*n+1,1)*yMean';
    Xd = X - ones(2*n+1,1)*xMean';
    Pyy = Yd'*W*Yd;
    Pxy = Xd'*W*Yd;
end

function [zHatNext, PzNext] = halfEkf(A, b, h, Q, R, r, u, zHat, Pz)
    % 予測ステップ
    zHatMinus = A * zHat + u;
    PzMinus = A * Pz * A' + Q*(b*b');
    
    % 更新ステップ
    % ヤコビアン行列の計算
    H = zeros(3,8);
    z1 = sqrt(zHatMinus(1)^2 + zHatMinus(2)^2);
    z2 = sqrt((zHatMinus(1)-zHatMinus(3))^2 + (zHatMinus(2)-zHatMinus(4))^2);
    z3 = sqrt(zHatMinus(3)^2 + zHatMinus(4)^2);
    
    % h1 = sqrt(x1^2 + y1^2)
    H(1,1) = zHatMinus(1)/z1;
    H(1,2) = zHatMinus(2)/z1;
    
    % h2 = sqrt((x1 - x2)^2 + (y1 - y2)^2)
    H(2,1) = (zHatMinus(1) - zHatMinus(3))/z2;
    H(2,2) = (zHatMinus(2) - zHatMinus(4))/z2;
    H(2,3) = (zHatMinus(3) - zHatMinus(1))/z2;
    H(2,4) = (zHatMinus(4) - zHatMinus(2))/z2;
    
    % h3 = sqrt(x2^2 + y2^2)
    H(3,3) = zHatMinus(3)/z3;
    H(3,4) = zHatMinus(4)/z3;
    % カルマンゲインの計算
    K = PzMinus * H' / (H * PzMinus * H' + R);

    % 状態と共分散の更新
    zHatNext = zHatMinus + K * (r - h(zHatMinus));
    PzNext = (eye(8) - K * H) * PzMinus;
end