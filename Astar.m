% A* simulering 
clc; clear;
image = imread('C:\Users\Tilla\OneDrive\Bilder\sistekart.png');

grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
% 
grid = binaryOccupancyMap(bwimage);
% 
planner = plannerAStarGrid(grid)

% show(grid)

hold on
start = [600 370];
goal = [170 540];

%plan(planner,start1,goal1);




plan(planner,start,goal);

%plan(planner,start2,goal2);

show(planner)

% goal = [700 700];
% 
% % start = [2 3];
% % goal = [10 1600];
% 
% plan(planner,start,goal);
% 
% show(planner)


% hold on
% x1 = 370;
% y1 = 280;
% plot(x1,y1,'g*')
% 
% x2 = 560;
% y2 = 690;
% plot(x2,y2,'g*')
% 
% x3 = 770;
% y3 = 520;
% plot(x3,y3,'g*')
% 
% x4 = 990;
% y4 = 360;
% plot(x4,y4,'g*')
% 
% x5 = 1200;
% y5 = 460;
% plot(x5,y5,'g*')
% 
% x6 = 1260;
% y6 = 140;
% plot(x6,y6,'g*')
% 
% x7 = 1420;
% y7 = 380;
% plot(x7,y7,'g*')
% 
% x8 = 1570;
% y8 = 580;
% plot(x8,y8,'g*')
% 
% %  p = Lattice(grid,'iterations', 2)
% % 
% %  p.plan('iterations', 2)
% % 
% %  figure;
% %  p.plot()
% % 
% %  p.query([560 690],[770 520])
% % 
% % p.plot()
% 
% % lp = Lattice(grid,'grid', 4, 'root', [4 4 0], 'inflate', 2);
% % lp.plan('iterations', 60, 'cost', [1 20 20])
% % lp2 = Lattice(grid, 'grid', 4, 'root', [4 4 0], 'inflate', 2);
% % save lp
% % save lp2
% % lp.plot()
% 
