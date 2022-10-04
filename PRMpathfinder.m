% PRM 
clc; clear;

% Importer bilde 
image = imread('C:\Users\Tilla\OneDrive\Bilder\ferdigkart1.png');

% Konverter til grayscale og så til svart hvit bilde basert på terskelverdi 
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% Bruk svar hvit bilde som et matrise inout til binært occupancy grid. 
map = binaryOccupancyMap(bwimage);
show(map)


% For at roboten ikke skal kollidere med hindringer, må man 
% blåse oppp kartet med robotens dimensjoner. 

% Robotens radius 
robotRadius = 0.2;

% Blåser opp kartet 
mapInflated = copy(map);
inflate(mapInflated,robotRadius);

% Definere path planner
prm = mobileRobotPRM;
prm.Map = mapInflated;

% Antall noder 
prm.NumNodes = 1500; % For lite med 1000

% Maks avstand mellom nodene  
prm.ConnectionDistance = 100;  

% Start og målpunkt 
start = [550 680];
goal = [750 490];

% Finner path mellom start og målpunkt
% ved å bruke findpath funksjonen 
path = findpath(prm, start, goal)
% Får ut et sett med start og målpunkt (mellom nodene) 

% Display 
show(prm)
