%clear
clc
close all
format compact;

%    T0 = csvread('SLS_SPEED_NEW4/sls_speed_sphere_76.2_169.txt');
%    T1 = csvread('SLS_SPEED_NEW4/sls_speed_sphere_101.6_155.txt');
%    T2 = csvread('SLS_SPEED_NEW4/sls_speed_sphere_127_124.txt');
%    T3 = csvread('SLS_SPEED_NEW4/sls_speed_sphere_152.4_103.txt');
%    T4 = csvread('SLS_SPEED_NEW4/sls_speed_sphere_177.8_89.txt');
%
%    T1 = csvread('SLS_SPEED_NEW4/sls_speed_mixed_101.6_108.txt');
%    T2 = csvread('SLS_SPEED_NEW4/sls_speed_mixed_127_107.txt');
%    T3 = csvread('SLS_SPEED_NEW4/sls_speed_mixed_152.4_103.txt');
%    T4 = csvread('SLS_SPEED_NEW4/sls_speed_mixed_177.8_83.txt');
%
%    T1 = csvread('SLS_SPEED_NEW3/sls_speed_mixed_101.6_147.txt');
%    T2 = csvread('SLS_SPEED_NEW3/sls_speed_mixed_127_117.txt');
%    T3 = csvread('SLS_SPEED_NEW3/sls_speed_mixed_152.4_98.txt');
%    T4 = csvread('SLS_SPEED_NEW3/sls_speed_mixed_177.8_88.txt');
%
%    [rows(1) cols(1)] = size(T1);
%    [rows(2) cols(2)] = size(T2);
%    [rows(3) cols(3)] = size(T3);
%    [rows(4) cols(4)] = size(T4);
%    TT1 = [T1;zeros( max(rows)-rows(1),15)];
%    TT2 = [T2;zeros( max(rows)-rows(2),15)];
%    TT3 = [T3;zeros( max(rows)-rows(3),15)];
%    TT4 = [T4;zeros( max(rows)-rows(4),15)];
%    T = cat(3, TT1,TT2,TT3,TT4);

max_fun = @(x)max(x)

load('NEW4_S.mat');
r = size(T,1);
speeds = ['101.6 mm/s'; '127.0 mm/s'; '152.4 mm/s'; '177.8 mm/s'];
speeds_str = cellstr(speeds);
figure
for  i =1:4
    xx = T(20:r,1,i);
    yy = T(20:r,3,i);
    zz = T(20:r,2,i);
    
    x_data = [];
    y_data = [];
    z_data = [];
    bounds_x = [-2.5 2.5];
    bounds_y = [10 15];
    bounds_z = [.2 .5];
    count = 1;
    for j = 1:size(xx)
        if(xx(j)>bounds_x(1)  && xx(j) <bounds_x(2))
            if(yy(j)>bounds_y(1) && yy(j) <bounds_y(2))
                if(zz(j)>bounds_z(1) && zz(j)< bounds_z(2) )
                    x_data(count) = xx(j);
                    y_data(count) = yy(j);
                    z_data(count) = zz(j);
                end
                count = count +1;
            end
        end
    end
    clear xx yy zz;
    
    subplot(2, 2, i);
    tx = bounds_x(1):.2:bounds_x(2);
    ty = bounds_y(1):.2:bounds_y(2);
    [gx,gy] = meshgrid(tx,ty);
    grid_centers = [gx(:),gy(:)];
    kdtreeobj = KDTreeSearcher(grid_centers);
    accum_indicies = kdtreeobj.knnsearch([x_data',y_data']); % # classification
    
    results = accumarray(accum_indicies,z_data',[],max_fun );
    results(results<bounds_z(1))=NaN;
    mn = mean(results);
    sd = std(results);
    results(results>mn+5*sd)=NaN;
    results(results<mn-5*sd)=NaN;
    dist = fitdist(results, 'beta');
    normal_data = random(dist, size(results,1),1);
    %hold on
    %histfit(normal_data,25,'beta')
    hist(results(:),100)
    alpha(0.5);
    %bar(hist(results(:),25) ./ sum(hist(results(:),25)))
    %hold off
    title(speeds_str(i));
    [skewness(results(:)) skewness(normal_data(:)) ; kurtosis(results(:)) kurtosis(normal_data(:))]
    skew(i) = skewness(normal_data(:));
    kurt(i) = kurtosis (normal_data(:));
end
figure
subplot(1, 2, 1);
plot(skew);
title('skewness');
subplot(1, 2, 2);
plot(kurt);
title('kurtosis');





