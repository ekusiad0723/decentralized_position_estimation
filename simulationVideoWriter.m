% filepath: /C:/Users/daisu/Documents/GitHub/decentralized_position_estimation/simulationVideoWriter.m
classdef simulationVideoWriter < handle
    properties
        FrameRate
        TimesSpeed
        SimulationTime
        SpaceSize
        WindowPosition
        FilePath
        StaticObjects
        DynamicObjects
    end
    
    methods
        function obj = simulationVideoWriter(frameRate, timesSpeed, simulationTime, spaceSize, windowPosition, filePath)
            obj.FrameRate = frameRate;
            obj.TimesSpeed = timesSpeed;
            obj.SimulationTime = simulationTime;
            obj.SpaceSize = spaceSize;
            obj.WindowPosition = windowPosition;
            obj.FilePath = filePath;
            obj.StaticObjects = {};
            obj.DynamicObjects = {};
        end
        
        function addStaticObject(obj, x, y, color, marker)
            obj.StaticObjects{end+1} = struct('x', x, 'y', y, 'color', color, 'marker', marker);
        end
        
        function addDynamicObject(obj, xData, yData, color, marker)
            obj.DynamicObjects{end+1} = struct('xData', xData, 'yData', yData, 'color', color, 'marker', marker);
        end
        
        function writeVideo(obj, t)
            v = VideoWriter(obj.FilePath, 'MPEG-4');
            v.FrameRate = obj.FrameRate; % フレームレートを設定
            open(v);

            % グラフのサイズを設定
            figure('Name', 'Simulation Video', 'Position', obj.WindowPosition); % 720pの解像度に設定
            hold on;
            xlabel('x');
            ylabel('y');
            title('Satellite Trajectory');
            xlim(obj.SpaceSize); % x軸の範囲を設定
            ylim(obj.SpaceSize); % y軸の範囲を設定
            axis equal; % アスペクト比を保持

            % 初期位置をプロット
            for i = 1:length(obj.StaticObjects)
                scatter(obj.StaticObjects{i}.x, obj.StaticObjects{i}.y, 100, obj.StaticObjects{i}.color, obj.StaticObjects{i}.marker);
            end

            % アニメーションのプロット
            numDynamicObjects = length(obj.DynamicObjects);
            dynamicHandles = gobjects(numDynamicObjects, 1);
            for i = 1:length(obj.DynamicObjects)
                dynamicHandles(i) = scatter(obj.DynamicObjects{i}.xData(1), obj.DynamicObjects{i}.yData(1), 100, obj.DynamicObjects{i}.color, obj.DynamicObjects{i}.marker);
            end

            % 軌跡をプロット
            trajHandles = gobjects(numDynamicObjects, 1);
            for i = 1:length(obj.DynamicObjects)
                trajHandles(i) = plot(obj.DynamicObjects{i}.xData(1), obj.DynamicObjects{i}.yData(1), obj.DynamicObjects{i}.color);
            end

            % 判例を追加
            legend({'Satellite 1 Initial Position', 'Satellite 2 Initial Position', 'Satellite 3 Initial Position', ...
                    'Satellite 1 Current Position', 'Satellite 2 Current Position', 'Satellite 3 Current Position', ...
                    'Satellite 1 Trajectory', 'Satellite 2 Trajectory', 'Satellite 3 Trajectory'}, ...
                    'Location', 'northeastoutside');

            % 倍速を表示
            text(1.05, 0.5, sprintf('%dx Speed', obj.TimesSpeed), 'Units', 'normalized', 'FontSize', 12);

            % 現在時刻の表示
            timeText = text(1.05, 0.45, sprintf('t = %.2f s', 0), 'Units', 'normalized', 'FontSize', 12);

            % アニメーションの更新
            videoSteps = obj.SimulationTime * obj.FrameRate / obj.TimesSpeed; % 動画のフレーム数
            index = 1; % 時間ベクトルのインデックス
            for i = 1 : videoSteps
                index = obj.findNearTimeIndex(t, i/obj.FrameRate*obj.TimesSpeed, index);
                for j = 1:numDynamicObjects
                    set(dynamicHandles(j), 'XData', obj.DynamicObjects{j}.xData(index), 'YData', obj.DynamicObjects{j}.yData(index));
                    set(trajHandles(j), 'XData', obj.DynamicObjects{j}.xData(1:index), 'YData', obj.DynamicObjects{j}.yData(1:index));
                end

                % 現在時刻を更新
                set(timeText, 'String', sprintf('t = %.2f s', t(index)));

                drawnow; % プロットを即座に更新
                frame = getframe(gcf); % 現在のフレームを取得
                writeVideo(v, frame); % フレームを動画ファイルに書き込む
            end

            hold off;

            % 動画ファイルを閉じる
            close(v);
        end
        
        function index = findNearTimeIndex(~, t, targetTime, startIndex)
            % 指定された時間に最も近いインデックスを見つける
            [~, index] = min(abs(t(startIndex:end) - targetTime));
            index = index + startIndex - 1;
        end
    end
end