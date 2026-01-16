function plot_error(ref_data, test_data, varargin)

    %% 1. 解析可选参数
    p = inputParser;
    addParameter(p, 'SaveFigs', false, @islogical);
    addParameter(p, 'SavePath', './figures/', @ischar);
    addParameter(p, 'DPI', 300, @isnumeric);
    addParameter(p, 'Format', 'png', @ischar);
    parse(p, varargin{:});
    
    save_figs = p.Results.SaveFigs;
    save_path = p.Results.SavePath;
    dpi_value = p.Results.DPI;
    fig_format = p.Results.Format;
    if save_figs && ~exist(save_path, 'dir'), mkdir(save_path); end

    %% 2. 数据处理
    t_ref = ref_data(:, 2);
    t_test = test_data(:, 2);
    [~, idx_ref, idx_test] = intersect(t_ref, t_test);
    
    ref = ref_data(idx_ref, :);
    test = test_data(idx_test, :);
    t = t_ref(idx_ref);

    % 角度归一化 (-180 to 180)
    for k = 9:11 
        ref(:, k) = mod(ref(:, k) + 180, 360) - 180;
        test(:, k) = mod(test(:, k) + 180, 360) - 180;
    end
    
    % 计算误差
    diff = test - ref;
    for k = 9:11 % 角度过零修正
        d = diff(:, k);
        d(d > 180) = d(d > 180) - 360;
        d(d < -180) = d(d < -180) + 360;
        diff(:, k) = d;
    end

    %% 3. 统计信息打印 (均值, RMS, 最大, 最小)
    var_names = {'Vn', 'Ve', 'Vd', 'Latitude', 'Longitude', 'Height', 'Roll', 'Pitch', 'Yaw'};
    units = {'m/s', 'm/s', 'm/s', 'deg', 'deg', 'm', 'deg', 'deg', 'deg'};
    cols = [6, 7, 8, 3, 4, 5, 9, 10, 11]; 
    
    fprintf('\n%s\n', repmat('=', 1, 80));
    fprintf('%-12s | %10s | %10s | %10s | %10s | %s\n', 'Parameter', 'Mean', 'RMS', 'Max', 'Min', 'Unit');
    fprintf('%s\n', repmat('-', 1, 80));
    for i = 1:9
        data = diff(:, cols(i));
        fprintf('%-12s | %10.3e | %10.3e | %10.3e | %10.3e | %s\n', ...
            var_names{i}, mean(data), rms(data), max(data), min(data), units{i});
    end
    fprintf('%s\n', repmat('=', 1, 80));

    %% 4. 绘制对比图 (Figure 1 - 解决遮挡问题)
    f1 = figure('Name', 'Comparison Plots', 'Color', 'w', 'Position', [100, 100, 1100, 750]);
    for i = 1:9
        subplot(3,3,i); hold on;
        % 根据行号选色 (Row1:红蓝, Row2:绿黑, Row3:品红青)
        if i <= 3,     c_our = [0.9 0.1 0.1]; c_ref = [0.1 0.4 0.9]; 
        elseif i <= 6, c_our = [0.2 0.7 0.2]; c_ref = [0.0 0.0 0.0]; 
        else,          c_our = [1.0 0.0 1.0]; c_ref = [0.0 0.8 0.8]; 
        end
        
        % 先画 Our 实线，后画 True 虚线，确保虚线能够“穿透”实线显示
        plot(t, test(:, cols(i)), '-', 'Color', c_our, 'LineWidth', 1.5, 'DisplayName', [var_names{i},' (Our)']);
        plot(t, ref(:, cols(i)), '--', 'Color', c_ref, 'LineWidth', 1.0, 'DisplayName', [var_names{i},' (True)']);
        
        grid on; box on; set(gca, 'TickDir', 'in', 'FontName', 'Times New Roman');
        ylabel([var_names{i}, ' (', units{i}, ')'], 'FontWeight', 'bold');
        if i > 6, xlabel('Time (s)'); end
        legend('show', 'Location', 'best', 'FontSize', 7);
    end

    %% 5. 绘制误差分布图 (Figure 2)
    f2 = figure('Name', 'Error Plots', 'Color', 'w', 'Position', [150, 150, 1100, 750]);
    for i = 1:9
        subplot(3,3,i);
        if i <= 3, c = [0.9 0.1 0.1]; elseif i <= 6, c = [0.2 0.7 0.2]; else, c = [1.0 0.0 1.0]; end
        
        plot(t, diff(:, cols(i)), 'Color', c, 'LineWidth', 1.0);
        title(sprintf('%s Error (RMS: %.2e)', var_names{i}, rms(diff(:, cols(i)))), 'FontSize', 9);
        grid on; box on; set(gca, 'TickDir', 'in', 'FontName', 'Times New Roman');
        ylabel(units{i});
        if i > 6, xlabel('Time (s)'); end
    end

    %% 6. 保存图片 (解决句柄报错问题)
    if save_figs
        path1 = fullfile(save_path, ['Comparison_Plot.', fig_format]);
        path2 = fullfile(save_path, ['Error_Plot.', fig_format]);
        % 显式传入句柄 f1, f2
        print(f1, path1, ['-d', fig_format], sprintf('-r%d', dpi_value));
        print(f2, path2, ['-d', fig_format], sprintf('-r%d', dpi_value));
        fprintf('\nImages saved to: %s\n', save_path);
    end
end