% Få inn bilde til kart 

% Importer bilde 
image = imread('C:\Users\Tilla\OneDrive\Bilder\ferdigkart1.png');

% Konverter til grayscale og så til svart hvit bilde basert på terskelverdi 
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% Bruk svar hvit bilde som et matrise inout til binært occupancy grid. 
map = binaryOccupancyMap(bwimage);
show(map)