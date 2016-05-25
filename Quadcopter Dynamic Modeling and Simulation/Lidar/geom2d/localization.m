clc
clear all
close all

load('map.mat');
map = map;
[r,c] = size(map);                              %# Get the matrix size
imagesc(map);            %# Plot the image
colormap(gray);                                %# Use a gray colormap
axis equal                                     %# Make axes grid sizes equal
% set(gca,'XTick',1:(c+1),'YTick',1:(r+1),...  %# Change some axes properties
%         'XLim',[1 c+1],'YLim',[1 r+1],...
%         'GridLineStyle','-','XGrid','on','YGrid','on');


% pos 75,75,0 with a scan of 0 to 2pi gives no contact.
Tl = se2(60,50,0*pi/2);


p = laserScanner(2*pi, 0.001, 50, Tl, map, 100, 100);

figure(2)
plot(p(:,1)/pi,p(:,2))