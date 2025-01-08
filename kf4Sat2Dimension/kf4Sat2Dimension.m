% filepath: /d:/GitHub/decentralized_position_estimation/kf4Sat2Dimension/kf4Sat2Dimension.m
% 正三角形を2つ組み合わせたひし形の維持とカルマンフィルタによる位置推定

% 定数の定義
SIMULATION_TIME = 10800;  % シミュレーション時間
L = 0.3;       % 目標距離
M = 0.5;       % 衛星の質量
KP = 0.000001; % Pゲイン
KD = 0.001;    % Dゲイン
SATURATION_LIMIT = 0.05e-6; % 飽和入力の制限 N

FRAME_RATE = 30; % ビデオのフレームレート
TIMES_SPEED = 1800; % 動画の時間の速度倍率
SPACE_SIZE = [-0.4, 0.4]; % 衛星の初期位置の範囲
INERTIAL_FRAME_SIMULATION_VIDEO_WINDOW_POSITION = [0, 100, 1280, 720]; % ウィンドウの位置 [left bottom width height]
SAT1_FRAME_SIMULATION_VIDEO_WINDOW_POSITION = [100, 150, 1280, 720]; % ウィンドウの位置 [left bottom width height]
FORCE_PLOT_WINDOW_POSITION = [200, 200, 1280, 720]; % ウィンドウの位置 [left bottom width height]

% 時系列モデル
% z(k+1) = A * z(k) + b * v(k) + u(k)
% r(k) = h(z(k)) + w(k)
TIME_STEP = 0.1; % タイムステップ
A = [eye(8), TIME_STEP * eye(8);...
     zeros(8,8), eye(8)];
b = [zeros(8,1); TIME_STEP/M * ones(8,1)];
h = @(z) [sqrt(z(1).^2 + z(2).^2); sqrt(z(3).^2 + z(4).^2);...
         sqrt(z(5).^2 + z(6).^2); sqrt(z(7).^2 + z(8).^2)];
Q = 0.001 * eye(16);        % プロセスノイズ分散
R = 0.0001 * eye(4);        % 測定ノイズ分散

% 衛星の初期位置 (ひし形)
xr1 = [-0.2 + 0.4 * rand; -0.2 + 0.4 * rand];
xr2 = [0.2 + 0.4 * rand; 0.2 + 0.4 * rand];
xr3 = [-0.2 + 0.4 * rand; 0.2 + 0.4 * rand];
xr4 = [0.2 + 0.4 * rand; -0.2 + 0.4 * rand];

disp("calculating satellite trajectory...");

% 衛星の初期状態 z = [x1, y1, x2, y2, x3, y3, x4, y4, vx1, vy1, vx2, vy2, vx3, vy3, vx4, vy4]
z0 = [xr1(1), xr1(2), xr2(1), xr2(2), xr3(1), xr3(2), xr4(1), xr4(2),...
       0, 0, 0, 0, 0, 0, 0, 0];

options = odeset("MaxStep", TIMES_SPEED/FRAME_RATE/2); % 最大ステップ幅を設定
[t, z] = ode45(@(t, z) odeFcn(t, z, KP, KD, M, L, SATURATION_LIMIT), [0, SIMULATION_TIME], z0, options);

% Position estimation using Kalmanfilter
zKf = [
    z(:,1:2) - z(:,5:6), z(:,3:4) - z(:,7:8), ...
    z(:,5:6) - z(:,1:2), z(:,7:8) - z(:,3:4)
];
u = [zeros(1,16);...
     zeros(size(z,1),8), diff(zKf(:,5:16),1,1)
]; 
Pz =  0.5 * eye(16); % 共分散行列の初期値
zHat0 = [zKf(1,1:8), zeros(1,8)];  % 状態推定値の初期値
zHat = [zHat0; zeros(size(z,1)-1, 16)];
for i = 1:size(z,1)-1
    r = h(zKf(i,:)) + sqrt(R) * randn(4,1);
    [zHatTemp, Pz] = halfEkf(A, b, h, Q, R, r, u(i,:)', zHat(i,:)', Pz);
    zHat(i+1,:) = zHatTemp';
end

disp("satellite trajectory calculated.");
disp("creating video...");

% 動画ファイルの設定
if ~exist('result', 'dir')
    mkdir('result');
end

% ビデオ作成コードをここに追加
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

function [zHatNext, PzNext] = halfEkf(A, b, h, Q, R, r, u, zHat, Pz)
    % 予測ステップ
    zHatMinus = A * zHat + u;
    PzMinus = A * Pz * A' + Q * (b * b');
    
    % 更新ステップ (EKF)
    H = jacobianH(zHatMinus);
    
    % カルマンゲインの計算
    K = PzMinus * H' / (H * PzMinus * H' + R);
    
    % 状態と共分散の更新
    zHatNext = zHatMinus + K * (r - h(zHatMinus));
    PzNext = (eye(length(zHat)) - K * H) * PzMinus;
end

function H = jacobianH(z)
    % ヤコビアン行列の計算
    H = zeros(4,16);
    for i = 1:4
        xi = z((i-1)*2 +1);
        yi = z((i-1)*2 +2);
        ri = sqrt(xi^2 + yi^2);
        H(i, (i-1)*2 +1) = xi / ri;
        H(i, (i-1)*2 +2) = yi / ri;
    end
end

function dzdt = odeFcn(t, z, KP, KD, M, L, SAT_LIMIT)
    % PD制御による力の計算
    dzdt = zeros(16,1);
    
    % 各衛星の制御
    for i = 1:4
        idx = (i-1)*4 +1;
        pos = z(idx:idx+1);
        vel = z(idx+2:idx+3);
        
        % 目標位置との誤差
        error = pos - getDesiredPosition(i, L);
        d_error = vel; % 速度誤差
        
        % PD制御
        force = -KP * error - KD * d_error;
        
        % 飽和制限
        force = max(min(force, SAT_LIMIT), -SAT_LIMIT);
        
        % 加速度の計算
        acc = force / M;
        
        % 状態更新
        dzdt(idx:idx+1) = vel;
        dzdt(idx+2:idx+3) = acc;
    end
end

function posDes = getDesiredPosition(i, L)
    % ひし形の目標位置を定義
    switch i
        case 1
            posDes = [0; 0];
        case 2
            posDes = [L; 0];
        case 3
            posDes = [0; L];
        case 4
            posDes = [-L; 0];
        otherwise
            posDes = [0; 0];
    end
end