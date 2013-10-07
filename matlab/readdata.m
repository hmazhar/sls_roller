clear
clc

M = csvread('270.txt');
%M = max(min(M,1),-5);

xx = M(10:371486,1);
yy = M(10:371486,3);
zz = M(10:371486,2);

%xx = max(min(xx,4),-4);
%yy = max(min(yy,4),-4);

zz = max(min(zz,.6),.3);
rad = M(10:371486,9);
minX = min(xx);
maxX = max(xx);
minY = min(yy);
maxY = max(yy);

targetSize = [40 150];

xxBin = round( (xx-minX)/(maxX-minX)*(targetSize(1)-1) ) +1;
yyBin = round( (yy-minY)/(maxY-minY)*(targetSize(2)-1) ) +1;

map = accumarray([xxBin(:),yyBin(:)],zz,targetSize,@max,0);

surf(map)


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