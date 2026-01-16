function [infile] = Read_simu_ref(filename)

    % 一次性读取
    fid = fopen(filename, 'rb');
    if fid == -1
        error('无法打开文件: %s', filename);
    end
    
    % 读取所有数据
    allData = fread(fid, [10, inf], 'double');
    fclose(fid);
    
    % 转置并添加第一列的0
    [~, numEpochs] = size(allData);
    infile = zeros(numEpochs, 11);
    infile(:, 1) = 0;
    infile(:, 2:11) = allData';

end