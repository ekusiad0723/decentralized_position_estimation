% filepath: /c:/Users/daisu/Documents/GitHub/decentralized_position_estimation/observable_analysis/saveFigureFHD.m
function saveFigureFHD(fig, fileNameBase, saveFolder)
    % saveFigureFHD - FHDサイズに固定してpngとsvg形式で保存する関数
    %
    % 入力:
    %   fig          : 保存対象のfigureハンドル
    %   fileNameBase : ファイル名のベース（拡張子なし）
    %   saveFolder   : 保存先のフォルダ（存在しなければ作成される）
    
    if ~exist(saveFolder, 'dir')
        mkdir(saveFolder);
    end
    
    % FHDサイズに固定（1920x1080ピクセル）
    set(fig, 'Units', 'pixels', 'Position', [0, 0, 1920, 1080]);
    
    % ファイル名の作成
    filename_png = fullfile(saveFolder, [fileNameBase, '.png']);
    filename_svg = fullfile(saveFolder, [fileNameBase, '.svg']);
    
    % png形式とsvg形式で出力
    print(fig, filename_png, '-dpng', '-r0');
    print(fig, filename_svg, '-dsvg');
end