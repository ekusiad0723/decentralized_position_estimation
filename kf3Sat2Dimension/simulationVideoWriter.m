% filepath: /c:/Users/daisu/Documents/GitHub/decentralized_position_estimation/kf3Sat2Dimension/simulationVideoWriter.m
classdef simulationVideoWriter < handle
    properties(SetAccess = private)
        FrameRate
        TimesSpeed
        SimulationTime
        SpaceSize
        Title
        WindowPosition
        FilePath
        Time
        StaticObjects
        DynamicObjects
    end
    
    methods
        function obj = simulationVideoWriter(frameRate, timesSpeed, simulationTime, spaceSize, title, windowPosition, filePath)
            obj.FrameRate = frameRate;
            obj.TimesSpeed = timesSpeed;
            obj.SimulationTime = simulationTime;
            obj.SpaceSize = spaceSize;
            obj.Title = title;
            obj.WindowPosition = windowPosition;
            obj.FilePath = filePath;
            obj.Time = [];
            obj.StaticObjects = {};
            obj.DynamicObjects = {};
        end

        function addTime(obj, time)
            obj.Time = time;
        end
        
        function addStaticObject(obj, x, y, color, marker, displayName)
            obj.StaticObjects{end+1} = struct('x', x, 'y', y, 'color', color, 'marker', marker, 'displayName', displayName);
        end
        
        function addDynamicObject(obj, xData, yData, color, marker, displayName, trajectoryDisplayName)
            obj.DynamicObjects{end+1} = struct('xData', xData, 'yData', yData, 'color', color, 'marker', marker, 'displayName', displayName, 'trajectoryDisplayName', trajectoryDisplayName);
        end
        
        function writeVideo(obj)
            v = VideoWriter(obj.FilePath, 'MPEG-4');
            v.FrameRate = obj.FrameRate; 
            open(v);

            figure('Position', obj.WindowPosition);
            hold on;
            xlabel('x');
            ylabel('y');
            title(obj.Title);
            xlim([obj.SpaceSize(1), obj.SpaceSize(2)]);
            ylim([obj.SpaceSize(1), obj.SpaceSize(2)]);
            axis equal;
            
            
            % x=0とy=0の軸を描画 (3倍の長さ)
            plot([0, 0], [3*obj.SpaceSize(1), 3*obj.SpaceSize(2)], 'k--', 'DisplayName', 'y-axis');
            plot([3*obj.SpaceSize(1), 3*obj.SpaceSize(2)], [0, 0], 'k--', 'DisplayName', 'x-axis');

            % 初期位置をプロット
            for i = 1:length(obj.StaticObjects)
                scatter(obj.StaticObjects{i}.x, obj.StaticObjects{i}.y, 100, obj.StaticObjects{i}.color, ...
                    obj.StaticObjects{i}.marker, 'DisplayName', obj.StaticObjects{i}.displayName);
            end

            dynamicHandles = gobjects(1, length(obj.DynamicObjects));
            trajHandles = gobjects(1, length(obj.DynamicObjects));
            for i = 1:length(obj.DynamicObjects)
                dynamicHandles(i) = scatter(obj.DynamicObjects{i}.xData(1), ...
                    obj.DynamicObjects{i}.yData(1), 100, obj.DynamicObjects{i}.color, ...
                    obj.DynamicObjects{i}.marker, 'DisplayName', obj.DynamicObjects{i}.displayName);
                trajHandles(i) = plot(obj.DynamicObjects{i}.xData(1), ...
                    obj.DynamicObjects{i}.yData(1), obj.DynamicObjects{i}.color, ...
                    'DisplayName', obj.DynamicObjects{i}.trajectoryDisplayName);
            end

            legend('Location','northeastoutside');

            videoSteps = obj.SimulationTime * obj.FrameRate / obj.TimesSpeed;
            index = 1;
            for i = 1 : videoSteps
                simTime = i / obj.FrameRate * obj.TimesSpeed;
                index = obj.findNearTimeIndex(obj.Time, simTime, index);

                for j = 1:length(dynamicHandles)
                    set(dynamicHandles(j), 'XData', obj.DynamicObjects{j}.xData(index), ...
                        'YData', obj.DynamicObjects{j}.yData(index));
                    set(trajHandles(j), 'XData', obj.DynamicObjects{j}.xData(1:index), ...
                        'YData', obj.DynamicObjects{j}.yData(1:index));
                end

                drawnow;
                frame = getframe(gcf);
                writeVideo(v, frame);
            end
            hold off;
            close(v);
        end
        
        function idx = findNearTimeIndex(~, t, targetTime, startIdx)
            [~, localIdx] = min(abs(t(startIdx:end) - targetTime));
            idx = startIdx + localIdx - 1;
        end
    end
end