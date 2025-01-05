TIME_STEP = 0.1; % time step
M = 1;
A = [eye(4),TIME_STEP * eye(4);...
    zeros(4,4),eye(4)];
b = [0; 0; 0; 0; TIME_STEP/M; TIME_STEP/M; TIME_STEP/M; TIME_STEP/M];
h = @(z) [sqrt(z(1).^2 + z(2).^2); sqrt(z(3).^2 + z(4).^2)];
Q = 0;        % process noise variance
R = 0;       % measurement noise variance

% テストデータの設定
r = [0.1834; 0.1087];

u = zeros(8,1);
u(5:8) = 1.0e-07 * [0.0187; -0.0000; -0.1813; -0.2000];

zHat = [0.0386; 0.0173; -0.0174; -0.0143; -0.0295; -0.0132; 0.0156; 0.0128];

Pz = [
    0.7376   -0.0033    0.0010    0.0008    0.3485   -0.0006    0.0002    0.0001;
   -0.0033    0.7435    0.0004    0.0004   -0.0006    0.3497    0.0001    0.0001;
    0.0010    0.0004    0.7432   -0.0015    0.0002    0.0001    0.3496   -0.0003;
    0.0008    0.0004   -0.0015    0.7438    0.0001    0.0001   -0.0003    0.3498;
    0.3485   -0.0006    0.0002    0.0001    0.4996   -0.0002    0.0000    0.0000;
   -0.0006    0.3497    0.0001    0.0001   -0.0002    0.4999    0.0000    0.0000;
    0.0002    0.0001    0.3496   -0.0003    0.0000    0.0000    0.4999   -0.0001;
    0.0001    0.0001   -0.0003    0.3498    0.0000    0.0000   -0.0001    0.4999
];

[zHatOut, PzOut] = halfUkf(A, b, h, Q, R, r, u, zHat, Pz);

function [zHatNext, PzNext] = halfUkf(A, b, h, Q, R, r, u, zHat, Pz)
    % 時系列モデル
    % z(k+1) = A * z(k) + b * v(k) + u(k)
    % r(k) = h(z(k)) + w(k)
    % 予測ステップ
    zHatMinus = A * zHat + u
    PzMinus = A * Pz * A' 
    % 更新ステップ
    [rHatMinus, Prr, Prz] = unscentedTransform(h, zHatMinus, PzMinus)
    g = Prz /(Prr + R*eye(2)) % 観測値は他の衛星の距離なので独立なノイズが乗ると考えられる
    zHatNext = zHatMinus + g * (r - rHatMinus)
    PzNext = PzMinus - g * Prz'
    disp(g * Prz')
end

function [yMean, Pyy, Pxy] = unscentedTransform(f, xMean, Pxx)
    % yMean = function(xMean)に対するU変換
    mapcols = @(f, x) cell2mat(cellfun(f,mat2cell(x,size(x,1),ones(1,size(x,2))),'UniformOutput',false));
    n = length(xMean);
    kappa = 0;
    w0 = kappa/(n+kappa);
    wi = 1/(2*(n+kappa));
    W = diag([w0;wi*ones(2*n,1)]);
    L = chol(Pxx)
    L*L'
    X = [xMean'; ones(n,1)*xMean' + sqrt(n+kappa)*L; ones(n,1)*xMean' - sqrt(n+kappa)*L]
    Y = mapcols(f, X')'
    yMean = sum(W*Y)'
    Yd = Y - ones(2*n+1,1)*yMean'
    Xd = X - ones(2*n+1,1)*xMean'
    Pyy = Yd'*W*Yd
    Pxy = Xd'*W*Yd
end