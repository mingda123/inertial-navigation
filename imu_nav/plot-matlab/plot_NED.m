function [] = plot_NED(varargin)
% 绘制平面/三维轨迹，支持多组数据对比和误差分析

    %% 1. 参数解析
    % 提取数据参数和选项参数
    data_cells = {};
    options_start = length(varargin) + 1;
    
    for i = 1:length(varargin)
        if ischar(varargin{i}) || isstring(varargin{i})
            options_start = i;
            break;
        else
            data_cells{end+1} = varargin{i};
        end
    end
    
    % 检查是否有输入数据
    if isempty(data_cells)
        error('至少需要输入一组轨迹数据！');
    end
    
    num_datasets = length(data_cells);
    
    % 解析选项参数
    p = inputParser;
    addParameter(p, 'PlotType', 'Both', @(x) any(strcmpi(x, {'2D', '3D', 'Both'})));
    addParameter(p, 'Labels', [], @iscell);
    addParameter(p, 'SaveFigs', false, @islogical);
    addParameter(p, 'SavePath', './figures/', @ischar);
    addParameter(p, 'DPI', 300, @isnumeric);
    addParameter(p, 'Format', 'png', @ischar);
    
    if options_start <= length(varargin)
        parse(p, varargin{options_start:end});
    else
        parse(p);
    end
    
    plot_type = p.Results.PlotType;
    labels = p.Results.Labels;
    save_figs = p.Results.SaveFigs;
    save_path = p.Results.SavePath;
    dpi_value = p.Results.DPI;
    fig_format = p.Results.Format;
    
    % 设置默认标签
    if isempty(labels)
        if num_datasets == 1
            labels = {'轨迹'};
        else
            labels = cell(1, num_datasets);
            labels{1} = '参考轨迹';
            for i = 2:num_datasets
                labels{i} = sprintf('测试轨迹%d', i-1);
            end
        end
    end
    
    % 创建保存目录
    if save_figs && ~exist(save_path, 'dir')
        mkdir(save_path);
    end
    
    %% 2. 坐标转换 (LLA -> ENU)
    GRS80.a = 6378137.0;
    GRS80.b = 6356752.3141;
    GRS80.e = sqrt(GRS80.a * GRS80.a - GRS80.b * GRS80.b) / GRS80.a;
    GRS80.e2 = GRS80.e * GRS80.e;
    
    % 首先进行数据对齐（使用时间戳）
    if num_datasets > 1
        % 提取所有数据集的时间戳
        time_stamps = cell(1, num_datasets);
        for k = 1:num_datasets
            time_stamps{k} = data_cells{k}(:, 2);  % 假设第2列是时间戳
        end
        
        % 找到所有数据集的公共时间点
        common_times = time_stamps{1};
        for k = 2:num_datasets
            common_times = intersect(common_times, time_stamps{k});
        end
        
        if isempty(common_times)
            warning('未找到公共时间点，将使用原始数据但可能导致对齐误差！');
            % 如果没有公共时间点，使用原始数据
        else
            % 根据公共时间点提取对齐后的数据
            aligned_data_cells = cell(1, num_datasets);
            for k = 1:num_datasets
                [~, idx] = ismember(common_times, time_stamps{k});
                aligned_data_cells{k} = data_cells{k}(idx, :);
            end
            data_cells = aligned_data_cells;
            fprintf('数据对齐完成：共有 %d 个公共时间点\n', length(common_times));
        end
    end
    
    % 转换所有数据集
    ENU_data = cell(1, num_datasets);
    for k = 1:num_datasets
        tmp_res = data_cells{k};
        tmp_res(:,3) = tmp_res(:,3) * pi / 180.0;  % 纬度转弧度
        tmp_res(:,4) = tmp_res(:,4) * pi / 180.0;  % 经度转弧度
        
        % 使用第一组数据的起点作为参考点
        if k == 1
            start_point = tmp_res(1, [3,4,5]);
        end
        
        n = size(tmp_res, 1);
        E = zeros(n, 1);
        N = zeros(n, 1);
        U = tmp_res(:, 5) - start_point(3);  % 高度相对于起点
        
        for i = 1:n
            Rm = GRS80.a * (1 - GRS80.e2) / sqrt((1 - GRS80.e2 * sin(tmp_res(i,3))^2)^3);
            Rn = GRS80.a / sqrt(1 - GRS80.e2 * sin(tmp_res(i,3))^2);
            N(i) = (tmp_res(i,3) - start_point(1)) * (Rm + tmp_res(i,5));
            E(i) = (tmp_res(i,4) - start_point(2)) * (Rn + tmp_res(i,5)) * cos(tmp_res(i,3));
        end
        
        ENU_data{k} = [E, N, U];
    end
    
    %% 3. 定义颜色方案
    colors = [
        0.2, 0.4, 0.8;   % 蓝色 - 参考
        0.9, 0.2, 0.3;   % 红色 - 测试1
        0.2, 0.8, 0.2;   % 绿色 - 测试2
        1.0, 0.5, 0.0;   % 橙色 - 测试3
        0.8, 0.2, 0.8;   % 紫色 - 测试4
        0.0, 0.8, 0.8;   % 青色 - 测试5
        0.8, 0.8, 0.2;   % 黄色 - 测试6
    ];
    
    line_styles = {'-', '--', '-.', ':', '-', '--', '-.'};
    line_width = 1.8;
    
    %% 4. 绘制2D轨迹图
    if strcmpi(plot_type, '2D') || strcmpi(plot_type, 'Both')
        fig_2d = figure('Name', '2D轨迹对比', 'Color', 'w', 'NumberTitle', 'off');
        set(fig_2d, 'Position', [100, 100, 900, 700]);
        hold on;
        
        for k = 1:num_datasets
            color_idx = mod(k-1, size(colors,1)) + 1;
            style_idx = mod(k-1, length(line_styles)) + 1;
            
            plot(ENU_data{k}(:,1), ENU_data{k}(:,2), ...
                 'Color', colors(color_idx,:), ...
                 'LineStyle', line_styles{style_idx}, ...
                 'LineWidth', line_width, ...
                 'DisplayName', labels{k});
            
            % 标记起点和终点
            plot(ENU_data{k}(1,1), ENU_data{k}(1,2), 'o', ...
                 'MarkerSize', 10, 'MarkerFaceColor', colors(color_idx,:), ...
                 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            plot(ENU_data{k}(end,1), ENU_data{k}(end,2), 's', ...
                 'MarkerSize', 10, 'MarkerFaceColor', colors(color_idx,:), ...
                 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
        
        hold off;
        xlabel('东向距离 E (m)', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('北向距离 N (m)', 'FontSize', 12, 'FontWeight', 'bold');
        title('平面轨迹对比图 (○起点 □终点)', 'FontSize', 14, 'FontWeight', 'bold');
        legend('Location', 'best', 'FontSize', 10);
        grid on;
        axis equal;
        set(gca, 'LineWidth', 1.2, 'FontSize', 11);
        
        if save_figs
            save_figure(fig_2d, save_path, '2D轨迹对比', dpi_value, fig_format);
        end
    end
    
    %% 5. 绘制3D轨迹图
    if strcmpi(plot_type, '3D') || strcmpi(plot_type, 'Both')
        fig_3d = figure('Name', '3D轨迹对比', 'Color', 'w', 'NumberTitle', 'off');
        set(fig_3d, 'Position', [150, 150, 900, 700]);
        hold on;
        
        for k = 1:num_datasets
            color_idx = mod(k-1, size(colors,1)) + 1;
            style_idx = mod(k-1, length(line_styles)) + 1;
            
            plot3(ENU_data{k}(:,1), ENU_data{k}(:,2), ENU_data{k}(:,3), ...
                  'Color', colors(color_idx,:), ...
                  'LineStyle', line_styles{style_idx}, ...
                  'LineWidth', line_width, ...
                  'DisplayName', labels{k});
            
            % 标记起点和终点
            plot3(ENU_data{k}(1,1), ENU_data{k}(1,2), ENU_data{k}(1,3), 'o', ...
                  'MarkerSize', 10, 'MarkerFaceColor', colors(color_idx,:), ...
                  'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            plot3(ENU_data{k}(end,1), ENU_data{k}(end,2), ENU_data{k}(end,3), 's', ...
                  'MarkerSize', 10, 'MarkerFaceColor', colors(color_idx,:), ...
                  'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
        
        hold off;
        xlabel('东向距离 E (m)', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('北向距离 N (m)', 'FontSize', 12, 'FontWeight', 'bold');
        zlabel('高度变化 U (m)', 'FontSize', 12, 'FontWeight', 'bold');
        title('三维轨迹对比图 (○起点 □终点)', 'FontSize', 14, 'FontWeight', 'bold');
        legend('Location', 'best', 'FontSize', 10);
        grid on;
        view(45, 30);  % 设置视角
        set(gca, 'LineWidth', 1.2, 'FontSize', 11);
        
        if save_figs
            save_figure(fig_3d, save_path, '3D轨迹对比', dpi_value, fig_format);
        end
    end
    
    %% 6. 多组数据时绘制误差分析
    if num_datasets > 1
        % 数据已经在步骤2中对齐，直接使用
        ref_data = ENU_data{1};
        
        fprintf('\n========== 轨迹误差统计分析 ==========\n');
        
        for k = 2:num_datasets
            test_data = ENU_data{k};
            
            % 确保数据长度一致（理论上已对齐，但为保险起见再检查）
            n_ref = size(ref_data, 1);
            n_test = size(test_data, 1);
            
            if n_ref ~= n_test
                warning('数据长度不一致！参考: %d, 测试: %d。使用较短长度。', n_ref, n_test);
                n_min = min(n_ref, n_test);
                ref_aligned = ref_data(1:n_min, :);
                test_aligned = test_data(1:n_min, :);
            else
                ref_aligned = ref_data;
                test_aligned = test_data;
                n_min = n_ref;
            end
            
            % 计算误差
            diff_ENU = test_aligned - ref_aligned;
            
            % 统计分析
            fprintf('\n--- %s vs %s ---\n', labels{k}, labels{1});
            fprintf('%-10s %12s %12s %12s %12s\n', '方向', '均值(m)', 'RMS(m)', '最大值(m)', '最小值(m)');
            fprintf('----------------------------------------------------------------\n');
            
            directions = {'东向(E)', '北向(N)', '高度(U)'};
            for i = 1:3
                fprintf('%-10s %12.4f %12.4f %12.4f %12.4f\n', ...
                        directions{i}, ...
                        mean(diff_ENU(:,i)), ...
                        rms(diff_ENU(:,i)), ...
                        max(diff_ENU(:,i)), ...
                        min(diff_ENU(:,i)));
            end
            
            % 计算水平误差和三维误差
            horizontal_error = sqrt(diff_ENU(:,1).^2 + diff_ENU(:,2).^2);
            spatial_error = sqrt(diff_ENU(:,1).^2 + diff_ENU(:,2).^2 + diff_ENU(:,3).^2);
            
            fprintf('%-10s %12.4f %12.4f %12.4f %12.4f\n', ...
                    '水平误差', ...
                    mean(horizontal_error), ...
                    rms(horizontal_error), ...
                    max(horizontal_error), ...
                    min(horizontal_error));
            fprintf('%-10s %12.4f %12.4f %12.4f %12.4f\n', ...
                    '空间误差', ...
                    mean(spatial_error), ...
                    rms(spatial_error), ...
                    max(spatial_error), ...
                    min(spatial_error));
            
            % 绘制误差图
            fig_err = figure('Name', sprintf('%s误差分析', labels{k}), ...
                           'Color', 'w', 'NumberTitle', 'off');
            set(fig_err, 'Position', [200+50*(k-1), 200+50*(k-1), 900, 800]);
            
            % 东向误差
            subplot(4,1,1);
            plot(1:n_min, diff_ENU(:,1), 'Color', [0.9, 0.2, 0.3], 'LineWidth', 1.5);
            ylabel('误差 (m)', 'FontWeight', 'bold');
            title(sprintf('东向(E)误差 - 均值: %.3f, RMS: %.3f', ...
                  mean(diff_ENU(:,1)), rms(diff_ENU(:,1))));
            grid on;
            set(gca, 'LineWidth', 1.1);
            
            % 北向误差
            subplot(4,1,2);
            plot(1:n_min, diff_ENU(:,2), 'Color', [0.2, 0.4, 0.8], 'LineWidth', 1.5);
            ylabel('误差 (m)', 'FontWeight', 'bold');
            title(sprintf('北向(N)误差 - 均值: %.3f, RMS: %.3f', ...
                  mean(diff_ENU(:,2)), rms(diff_ENU(:,2))));
            grid on;
            set(gca, 'LineWidth', 1.1);
            
            % 高度误差
            subplot(4,1,3);
            plot(1:n_min, diff_ENU(:,3), 'Color', [0.2, 0.8, 0.2], 'LineWidth', 1.5);
            ylabel('误差 (m)', 'FontWeight', 'bold');
            title(sprintf('高度(U)误差 - 均值: %.3f, RMS: %.3f', ...
                  mean(diff_ENU(:,3)), rms(diff_ENU(:,3))));
            grid on;
            set(gca, 'LineWidth', 1.1);
            
            % 水平误差
            subplot(4,1,4);
            plot(1:n_min, horizontal_error, 'Color', [0.8, 0.2, 0.8], 'LineWidth', 1.5);
            ylabel('误差 (m)', 'FontWeight', 'bold');
            xlabel('采样点', 'FontWeight', 'bold');
            title(sprintf('水平误差 - 均值: %.3f, RMS: %.3f', ...
                  mean(horizontal_error), rms(horizontal_error)));
            grid on;
            set(gca, 'LineWidth', 1.1);
            
            if save_figs
                save_figure(fig_err, save_path, sprintf('%s误差分析', labels{k}), ...
                          dpi_value, fig_format);
            end
        end
        
        fprintf('===========================================\n\n');
    end
    
    if save_figs
        fprintf('所有图片已保存到: %s\n', save_path);
    end
end

%% 辅助函数：保存高质量图片
function save_figure(fig_handle, path, name, dpi, format)
    % 构建文件名
    filename = fullfile(path, [name, '.', format]);
    
    % 设置打印选项以提高质量
    fig_handle.PaperPositionMode = 'auto';
    fig_handle.PaperUnits = 'inches';
    fig_pos = fig_handle.PaperPosition;
    fig_handle.PaperSize = [fig_pos(3) fig_pos(4)];
    
    % 根据格式保存
    switch lower(format)
        case 'png'
            print(fig_handle, filename, '-dpng', sprintf('-r%d', dpi));
        case 'jpg'
            print(fig_handle, filename, '-djpeg', sprintf('-r%d', dpi));
        case 'pdf'
            print(fig_handle, filename, '-dpdf', '-bestfit');
        case 'eps'
            print(fig_handle, filename, '-depsc', sprintf('-r%d', dpi));
        case 'tiff'
            print(fig_handle, filename, '-dtiff', sprintf('-r%d', dpi));
        otherwise
            warning('不支持的格式: %s，使用PNG格式保存', format);
            print(fig_handle, filename, '-dpng', sprintf('-r%d', dpi));
    end
    
    fprintf('已保存: %s\n', filename);
end