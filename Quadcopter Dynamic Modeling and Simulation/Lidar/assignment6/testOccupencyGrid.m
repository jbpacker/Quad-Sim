clear all;

addpath(genpath('/Users/jpacker/Documents/MATLAB/Apps/rvctools'));
addpath(genpath('/Users/jpacker/Documents/MATLAB/Apps/geom2d'));

global occupencyGrid;
global rangeMax;

R = 500; C = 500;
map=zeros(R, C);
occupencyGrid = ones(R, C); %initialize as empty
Xmax = 150; Ymax = 150;
%test rectangular obstacle
Xsw=70; Ysw = 30;
Xne=Xsw + 30; Yne= Ysw + 20;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;
angleSpan = pi; angleStep = angleSpan/120; rangeMax = 200;
locCnt = 1;
for k=1:5
    Tl = se2([10+20*k  5 pi/2]);
    p = laserScanner(angleSpan, angleStep, rangeMax, Tl, map, Xmax, Ymax);  
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
        n = updateOccupencyGrid(angle, range, Tl, R, C, Xmax, Ymax);
    end
    locations(locCnt,:) = [10+20*k  5];
    locCnt = locCnt+1;
end

for l=1:3
    Tl = se2([10+20*5  5+30*l pi/2-pi/2*l]);
    p = laserScanner(angleSpan, angleStep, rangeMax, Tl, map, Xmax, Ymax);  
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
        n = updateOccupencyGrid(angle, range, Tl, R, C, Xmax, Ymax);
    end
    locations(locCnt,:) = [10+20*5  5+40*l];
    locCnt = locCnt+1;
end
probability = 1./(1+occupencyGrid);


imagesc(probability);
colorbar
colormap gray
% colormap(flipud(colormap))
hold on
plot(locations, 'kx');
axis equal