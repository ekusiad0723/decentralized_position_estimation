%左上にずれるのは、測定誤差が本来は正規分布性ではないため
%測定誤差が正規分布性であれば、UKFは正確に状態を推定できる

simulationTime = 10;
TIME_STEP = 0.01;

%initial state
xTrue0 = [0.5; 0; -0.5; 1];
xEst0 = zeros(4,1);


%calculate True state
tspan = 0:TIME_STEP:simulationTime;
[tSol,xTrue] = ode45(@(tSol, xTrue) odefunc(tSol, xTrue), tspan, xTrue0);

% 制御入力の設定
uDiscrete = zeros(size(tDiscrete,2),4);
for i = 1:length(tDiscrete)
    if tDiscrete(i) >= 5 && tDiscrete(i) <= 5.5
        uDiscrete(i,3) = -0.1;
    end
end


tDiscrete = 0:TIME_STEP:simulationTime;
xTrueDiscrete = interp1(tSol,xTrue,tDiscrete);


%ukf
obj = unscentedKalmanFilter(@stateFcn,@measFcn,xEst0);
obj.StateCovariance = 0.5*eye(4);
obj.ProcessNoise = 0.1*[zeros(2), zeros(2); zeros(2), 0.01*eye(2)];
obj.MeasurementNoise = 0.1;
xEst = zeros(4,length(tDiscrete));

for i = 1:length(tDiscrete)
    obj.predict(uDiscrete(i,:)');
    obj.correct([sqrt((1-xTrueDiscrete(i,1))^2 + (1-xTrueDiscrete(i,2))^2);...
                sqrt((1+xTrueDiscrete(i,1))^2 + (1+xTrueDiscrete(i,2))^2);...
                sqrt((1+xTrueDiscrete(i,1))^2 + (1-xTrueDiscrete(i,2))^2)]);
    xEst(:,i) = obj.State;
end


%amimation
v = simulationVideoWriter(30, 1, simulationTime, [-1 1], 'UKF', [0 0 800 800], 'test_nonlinear_KF.mp4'); % frameRate, timesSpeed, simulationTime, spaceSize, title, windowPosition, filePath
v.addTime(tDiscrete);
v.addStaticObject(1,1,'g','x','Measurement Point');
v.addStaticObject(-1,-1,'g','x','Measurement Point');
v.addStaticObject(-1,1,'g','x','Measurement Point');
v.addDynamicObject(xTrueDiscrete(:,1), xTrueDiscrete(:,2), 'r', 'o', '-', 'True State', 'True State Trajectory');
v.addDynamicObject(xEst(1,:), xEst(2,:), 'r', '.', ':', 'Ukf Estimation', 'Ukf Estimation Trajectory');
v.writeVideo();

function dxdt = odefunc(t,x)
    a = 0.1;
    b = 0.1;
    g = 9.81;
    dxdt = [x(3); x(4); -g*x(1)/(1 + 4*a^2*x(1)^2); -g*x(2)/(1 + 4*b^2*x(2)^2)];
end

function xNext = stateFcn(x, u)
    a = 0.1;
    b = 0.1;
    g = 9.81;
    TIME_STEP = 0.01;
    xNext = [x(1) + TIME_STEP * x(3);...
            x(2) + TIME_STEP * x(4);...
            x(3) - TIME_STEP * g * x(1)/(1 + 4*a.^2*x(1)^2);...
            x(4) - TIME_STEP * g * x(2)/(1 + 4*b.^2*x(2)^2)];
    xNext = xNext + u;
end

function y = measFcn(x)
    y = [sqrt((1-x(1))^2 + (1-x(2))^2);...
         sqrt((1+x(1))^2 + (1+x(2))^2);...
         sqrt((1+x(1))^2 + (1-x(2))^2)];
end
