%clear
%clc

%M = csvread('1200.txt');
%N = csvread('200.txt');
%M = max(min(M,1),-5);

xx = N(10:631966,1);
yy = N(10:631966,3);
zz = N(10:631966,2);

xx = max(min(xx,3),-3);
yy = max(min(yy,3),-3);

%zz = max(min(zz,.6),.1);
%rad = M(10:371486,9);
minX = min(xx);
maxX = max(xx);
minY = min(yy);
maxY = max(yy);

% x_bin_edges = minX:.1:maxX;
% y_bin_edges =  minY:.1:maxY;
% [average, stdev, centers, population, out_of_range] = binXYZonXY( x_bin_edges, y_bin_edges, xx, yy, zz, 0, 1 );

dim_x = 60;
dim_y = dim_x+2;

targetSize = [dim_x dim_y];

xxBin = round( (xx-minX)/(maxX-minX)*(targetSize(1)-1) ) +1;
yyBin = round( (yy-minY)/(maxY-minY)*(targetSize(2)-1) ) +1;
colormap(gray(128))
map = accumarray([xxBin(:),yyBin(:)],zz,targetSize,@max,0);
map = map(1:dim_x,2:dim_y-1);
map = map-min(min(map));
x_int = 0:.1:60;


map=interp2(map,2);
imwrite(map, 'hmap_before.png')

surf(map,'EdgeColor','none');
colorbar
%min_h = min(min(map))
%max_h = max(max(map))
%std_dev = std2(map)

% dy = .2;
% dx = .2;
% 
% 
% yidx=[min(yy):dy:max(yy)];
% xidx=[min(xx):dx:max(xx)];
% ZmapSum=zeros(length(yidx),length(xidx));
% ZmapIdx=zeros(size(ZmapSum));
% 
% [nx,binx] = histc(xx,xidx);
% [ny,biny] = histc(yy,yidx);
% %bin==0 means the value is out of range
% binx=binx+1; biny=biny+1;
% %binzero=( (binx==0) | (biny==0) );
% %binx(binzero) = [];
% %biny(binzero) = [];
% %xx(binzero) = [];
% %yy(binzero) = [];
% %zz(binzero) = [];
% 
% %binx and biny give their respective bin locations
% for i=1:1:length(xx)
%     ZmapSum(biny(i),binx(i))=max(ZmapSum(biny(i),binx(i)),zz(i));
%     ZmapIdx(biny(i),binx(i))=ZmapIdx(biny(i),binx(i))+1;
% end
% 
% Zmap=ZmapSum./ZmapIdx;
% 
% surf(Zmap)