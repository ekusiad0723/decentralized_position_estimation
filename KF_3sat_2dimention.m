SIMULATION_TIME = 60;  % simulation time
Q = 0;        % process noise variance
R = 0;       % measurement noise variance
L = 150;       % target distance
MASS = 1;         % mass of the satellite
KP = 0.01; % P gain
KD = 0.15;  % D gain
FRAME_RATE = 60; % frame rate of the video
DT_ERROR = 0.1;  %誤差更新時間
TIMES_SPEED = 5; %動画の時間の速度倍率

disp("calculating satellite trajectory...");

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
[t, z] = ode45(@(t, z) odeFcn(t, z, KP, KD, MASS, L, R, Q, errors(:, 1)), [0, SIMULATION_TIME], z0,  options);

disp("satellite trajectory calculated.");
disp("creating video...");

% 動画ファイルの設定
if ~exist('result', 'dir')
    mkdir('result'); % resultフォルダが存在しない場合は作成
end

% ファイル名の生成
dateStr = datestr(now, 'yyyymmdd');
fileIndex = 1;
while exist(fullfile('result', sprintf('%s_%d_satelliteTrajectory.mp4', dateStr, fileIndex)), 'file')
    fileIndex = fileIndex + 1;
end
videoFile = fullfile('result', sprintf('%s_%d_satelliteTrajectory.mp4', dateStr, fileIndex));

v = VideoWriter(videoFile, 'MPEG-4');
v.FrameRate = FRAME_RATE; % フレームレートを設定
open(v);

% グラフのサイズを設定
figure('Position', [100, 100, 1280, 720]); % 720pの解像度に設定
hold on;
xlabel('x');
ylabel('y');
title('Satellite Trajectory');
xlim([-300, 300]); % x軸の範囲を設定
ylim([-200, 200]); % y軸の範囲を設定
axis equal; % アスペクト比を保持

% 初期位置をプロット
scatter(z(1, 1), z(1, 2), 100, 'r', 'x'); % 衛星1の初期位置
scatter(z(1, 3), z(1, 4), 100, 'g', 'x'); % 衛星2の初期位置
scatter(z(1, 5), z(1, 6), 100, 'b', 'x'); % 衛星3の初期位置

% アニメーションのプロット
h1 = scatter(z(1, 1), z(1, 2), 100, 'r', 'o'); % 衛星1の現在位置
h2 = scatter(z(1, 3), z(1, 4), 100, 'g', 'o'); % 衛星2の現在位置
h3 = scatter(z(1, 5), z(1, 6), 100, 'b', 'o'); % 衛星3の現在位置

% 軌跡をプロット
h1Traj = plot(z(1, 1), z(1, 2), 'r'); % 衛星1の軌跡
h2Traj = plot(z(1, 3), z(1, 4), 'g'); % 衛星2の軌跡
h3Traj = plot(z(1, 5), z(1, 6), 'b'); % 衛星3の軌跡

% 判例を追加
legend({'Satellite 1 Initial Position', 'Satellite 2 Initial Position', 'Satellite 3 Initial Position', ...
        'Satellite 1 Current Position', 'Satellite 2 Current Position', 'Satellite 3 Current Position', ...
        'Satellite 1 Trajectory', 'Satellite 2 Trajectory', 'Satellite 3 Trajectory'}, ...
        'Location', 'northeastoutside');

% 倍速を表示
text(1.05, 0.5, sprintf('%dx Speed', TIMES_SPEED), 'Units', 'normalized', 'FontSize', 12);

% アニメーションの更新
videoSteps = SIMULATION_TIME * FRAME_RATE / TIMES_SPEED; % 動画のフレーム数
index = 1; % 時間ベクトルのインデックス
for i = 1 : videoSteps
    index = findNearTimeIndex(t, i/FRAME_RATE*TIMES_SPEED, index);
    set(h1, 'XData', z(index, 1), 'YData', z(index, 2)); % 衛星1の現在位置を更新
    set(h2, 'XData', z(index, 3), 'YData', z(index, 4)); % 衛星2の現在位置を更新
    set(h3, 'XData', z(index, 5), 'YData', z(index, 6)); % 衛星3の現在位置を更新
    % 軌跡をプロット
    set(h1Traj, 'XData', z(1:index, 1), 'YData', z(1:index, 2)); % 衛星1の軌跡を更新
    set(h2Traj, 'XData', z(1:index, 3), 'YData', z(1:index, 4)); % 衛星2の軌跡を更新
    set(h3Traj, 'XData', z(1:index, 5), 'YData', z(1:index, 6)); % 衛星3の軌跡を更新
    
    drawnow; % プロットを即座に更新
    frame = getframe(gcf); % 現在のフレームを取得
    writeVideo(v, frame); % フレームを動画ファイルに書き込む
end

hold off;

% 動画ファイルを閉じる
close(v);

% グラフのウィンドウを閉じる
close(gcf);

disp("video created.");

% 制御力のみをフィードバック
function dydt = odeFcn(t, y, KP, KD, MASS, L, R, Q, errors)
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

function index = findNearTimeIndex(t, targetTime, startIndex)
    % 指定された時間に最も近いインデックスを見つける
    [~, index] = min(abs(t(startIndex:end) - targetTime));
    index = index + startIndex - 1;
    %disp("targetTime: " + targetTime + ", time: " + t(index) + ", index: " + index);
end