SIMULATION_TIME = 100;  % simulation time
Q = 0;        % process noise variance
R = 0;       % measurement noise variance
L = 150;       % target distance
MASS = 1;         % mass of the satellite
KP = 0.01; % P gain
KD = 0.15;  % D gain
FRAME_RATE = 30; % frame rate of the video
DT_ERROR = 0.1;  %誤差更新時間
TIMES_SPEED = 6; %動画の時間の速度倍率

disp("calculating satellite trajectory...");

% filepath: /C:/Users/daisu/Documents/GitHub/decentralized_position_estimation/KF_3sat_2dimention.m
% 衛星の初期位置
xr1 = [-100 + 200 * rand; -100 + 200 * rand];
xr2 = [-100 + 200 * rand; -100 + 200 * rand];
xr3 = [-100 + 200 * rand; -100 + 200 * rand];

%誤差を更新しながら、PD制御で衛星間の距離をLに制御
stepNum = ceil(SIMULATION_TIME/DT_ERROR);
% 誤差を先に生成
errors = randn(9, stepNum);


% 衛星の初期位置 状態変数z = [x1, y1, x2, y2, x3, y3, vx1, vy1, vx2, vy2, vx3, vy3]
z0 = [xr1(1), xr1(2), xr2(1), xr2(2), xr3(1), xr3(2), 0, 0, 0, 0, 0, 0];

options = odeset("MaxStep", TIMES_SPEED/FRAME_RATE/2); % 最大ステップ幅を設定 アニメーションがカクカクにならないように
[t, z] = ode45(@(t, z) odeFcn(t, z, KP, KD, MASS, L), [0, SIMULATION_TIME], z0,  options);

disp("satellite trajectory calculated.");
disp("creating video...");

% 動画ファイルの設定
if ~exist('result', 'dir')
    mkdir('result'); % resultフォルダが存在しない場合は作成
end

% ファイル名の生成
dateStr = datetime("now", "Format", "yyyyMMdd");
fileIndex = 1;
while exist(fullfile('result', sprintf('%s_%d_satelliteTrajectory.mp4', dateStr, fileIndex)), 'file')
    fileIndex = fileIndex + 1;
end
videoFile = fullfile('result', sprintf('%s_%d_satelliteTrajectory.mp4', dateStr, fileIndex));

% simulationVideoWriterクラスを使用して動画を書き出す
videoWriter = simulationVideoWriter(FRAME_RATE, TIMES_SPEED, SIMULATION_TIME, videoFile);

% 初期位置を追加
videoWriter.addStaticObject(z(1, 1), z(1, 2), 'r', 'x'); % 衛星1の初期位置
videoWriter.addStaticObject(z(1, 3), z(1, 4), 'g', 'x'); % 衛星2の初期位置
videoWriter.addStaticObject(z(1, 5), z(1, 6), 'b', 'x'); % 衛星3の初期位置

% 動的オブジェクトを追加
videoWriter.addDynamicObject(z(:, 1), z(:, 2), 'r', 'o'); % 衛星1の現在位置
videoWriter.addDynamicObject(z(:, 3), z(:, 4), 'g', 'o'); % 衛星2の現在位置
videoWriter.addDynamicObject(z(:, 5), z(:, 6), 'b', 'o'); % 衛星3の現在位置

videoWriter.writeVideo(t, z);

disp("video created.");

% 制御力のみをフィードバック
function dydt = odeFcn(t, y, KP, KD, MASS, L)
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
    dydt(7) = -KP/MASS*((L-r12)/r12*(y(3)-y(1)) + (L-r31)/r31*(y(5)-y(1))) - KD*y(7);
    dydt(8) = -KP/MASS*((L-r12)/r12*(y(4)-y(2)) + (L-r31)/r31*(y(6)-y(2))) - KD*y(8);
    dydt(9) = -KP/MASS*((L-r12)/r12*(y(1)-y(3)) + (L-r23)/r23*(y(5)-y(3))) - KD*y(9);
    dydt(10) = -KP/MASS*((L-r12)/r12*(y(2)-y(4)) + (L-r23)/r23*(y(6)-y(4))) - KD*y(10);
    dydt(11) = -KP/MASS*((L-r23)/r23*(y(3)-y(5)) + (L-r31)/r31*(y(1)-y(5))) - KD*y(11);
    dydt(12) = -KP/MASS*((L-r23)/r23*(y(4)-y(6)) + (L-r31)/r31*(y(2)-y(6))) - KD*y(12);
end
