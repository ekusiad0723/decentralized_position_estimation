t = 300;  % simulation time
q = 0;        % process noise variance
r = 0;       % measurement noise variance
l = 150;       % target distance
mass = 1;         % mass of the satellite
kp = 0.01; % P gain
kd = 0.2;  % D gain
frameRate = 30; % frame rate of the video
dtError = 0.01;  %誤差更新時間
timesSpeed = 10; %動画の時間の速度倍率

disp("calculating satellite trajectory...");

% 衛星の初期位置
xr1 = [-100 + 200 * rand; -100 + 200 * rand];
xr2 = [-100 + 200 * rand; -100 + 200 * rand];
xr3 = [-100 + 200 * rand; -100 + 200 * rand];

%誤差を更新しながら、PD制御で衛星間の距離をlに制御
stepNum = ceil(t/dtError);
% 誤差を先に生成
errors = randn(9, stepNum);
% 衛星の初期位置 状態変数z = [x1, y1, x2, y2, x3, y3, vx1, vy1, vx2, vy2, vx3, vy3]
z0 = [xr1(1), xr1(2), xr2(1), xr2(2), xr3(1), xr3(2), 0, 0, 0, 0, 0, 0];
time = [];
z = [];
for i = 1:stepNum
    [tLocal, zLocal] = ode45(@(tLocal, zLocal) odeFcn(tLocal, zLocal, kp, kd, mass, l, r, q, errors(:, i)), [0, dtError], z0);
    tLocal = tLocal + dtError*(i-1)*ones(size(tLocal));
    time = [time; tLocal];
    z = [z; zLocal];
    z0 = z(end, :);
end

disp("satellite trajectory calculated.");
disp("creating video...");

% 動画ファイルの設定
if exist('satelliteTrajectory.mp4', 'file') == 2
    delete('satelliteTrajectory.mp4'); % 既存のファイルがあれば削除 この処理があるとVSCodeでエラーがなぜか出ない
end
videoFile = 'satelliteTrajectory.mp4'; % 相対パスに変更
v = VideoWriter(videoFile, 'MPEG-4');
v.FrameRate = frameRate; % フレームレートを設定
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
text(1.05, 0.5, sprintf('%dx Speed', timesSpeed), 'Units', 'normalized', 'FontSize', 12);

% アニメーションの更新
videoSteps = t * frameRate / timesSpeed; % 動画のフレーム数
index = 1; % 時間ベクトルのインデックス
for i = 1 : videoSteps
    index = findNearTimeIndex(time, i/frameRate*timesSpeed, index);
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

function dydt = odeFcn(t, y, kp, kd, mass, l, r, q, errors)
    dydt = zeros(12, 1);
    % 衛星の状態量
    dydt(1) = y(7);
    dydt(2) = y(8);
    dydt(3) = y(9);
    dydt(4) = y(10);
    dydt(5) = y(11);
    dydt(6) = y(12);
    % 衛星間の距離に観測ノイズを付加
    r12 = sqrt((y(1)-y(3))^2 + (y(2)-y(4))^2) + sqrt(r) * randn;
    r23 = sqrt((y(3)-y(5))^2 + (y(4)-y(6))^2) + sqrt(r) * randn;
    r31 = sqrt((y(5)-y(1))^2 + (y(6)-y(2))^2) + sqrt(r) * randn;
    % 加速度フィードバック制御
    dydt(7) = -kp/mass*((l-r12)/r12*(y(3)-y(1)) + (l-r31)/r31*(y(5)-y(1))) - kd*y(7) + sqrt(q) * randn + errors(1);
    dydt(8) = -kp/mass*((l-r12)/r12*(y(4)-y(2)) + (l-r31)/r31*(y(6)-y(2))) - kd*y(8) + sqrt(q) * randn + errors(2);
    dydt(9) = -kp/mass*((l-r12)/r12*(y(1)-y(3)) + (l-r23)/r23*(y(5)-y(3))) - kd*y(9) + sqrt(q) * randn + errors(3);
    dydt(10) = -kp/mass*((l-r12)/r12*(y(2)-y(4)) + (l-r23)/r23*(y(6)-y(4))) - kd*y(10) + sqrt(q) * randn + errors(4);
    dydt(11) = -kp/mass*((l-r23)/r23*(y(3)-y(5)) + (l-r31)/r31*(y(1)-y(5))) - kd*y(11) + sqrt(q) * randn + errors(5);
    dydt(12) = -kp/mass*((l-r23)/r23*(y(4)-y(6)) + (l-r31)/r31*(y(2)-y(6))) - kd*y(12) + sqrt(q) * randn + errors(6);
end

function index = findNearTimeIndex(t, targetTime, startIndex)
    % 指定された時間に最も近いインデックスを見つける
    [~, index] = min(abs(t(startIndex:end) - targetTime));
    index = index + startIndex - 1;
end