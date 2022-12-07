 clc; 
 clear all;

%% Lager kart ved å kjøre eget skript 
%Får også tilgang på punktene den skal innom 
kjopesenter_skript;

%% Legger inn egenskaper til H2rObot 
maksStyresvinkel=deg2rad(40);
breddeRobot=0.70;
wheelbase=0.60;
%
VehicleSpeedRange=[0 0.5];

%% Lattice planner 
%Disse må kommenteres vekk første gang man kjører. 
% %Skal koste mer å svinge enn å kjøre rett
% lp=Lattice(kjopesenter,'grid',3,'root',[80,30,0],'inflate',1,'cost',[1,10,10]) 
% lp.plan('cost',[1,10,10])
% lp2=Lattice(kjopesenter,'grid',3,'root',[80,30,0],'inflate',1,'cost',[1,10,10]) 
% save lp
% save lp2
% lp.plot()
%% Henter lagret lattice planner 
load lp
lp.plot()
%% Planlegging av rute 
%Finner rute fra start posisjon til slutt posisjon 
p=lp.query(P2,P3)                                   
%Henter ut x og y kordinatene
p1 = p(:,1);
p2 = p(:,2);

%Setter x og y kordinantene sammen til en matrise  
path = [p1,p2]/10;
path_plot = path*10;
%% Controller - PurePursuit 
%Med bisyclemodel 

%Setter start og slutt posisjon for kontroller
robotStartPos = path(1,:);
robotMaalpos = path(end,:);
initialOrientation = 0;
%Setter start Pose til den mobile basen
robotCurrentPose = [robotStartPos initialOrientation]';
%Kinematisk modell for mobilbase 
robotBase = bicycleKinematics("WheelBase",wheelbase,"MaxSteeringAngle",maksStyresvinkel,"VehicleInputs","VehicleSpeedSteeringAngle");

%Viser planlagt rute til controller som en del av simulering 
figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 120])
ylim([0 80])

%% Robot skal kontrolleres via PurePursuit
H2rObot_controller = controllerPurePursuit;
%Gir kontrolleren banen den skal følge 
H2rObot_controller.Waypoints = path;

% Setter ønsket lineær hastighet -->%Krav på 0.5m/s
H2rObot_controller.DesiredLinearVelocity = 0.5;
% Setter svinghastighet / vinkelhastighet
H2rObot_controller.MaxAngularVelocity = 10;
% Setter hvor langt frem kontrolleren skal se for å beregne Pure Pursuit
H2rObot_controller.LookaheadDistance = 0.5;
% Hvor nært mål vi ønsker å komme 
goalRadius = 0.1;
%Setter avstand til mål
distanceToGoal = norm(robotStartPos - robotMaalpos);

%% Simulering
% Setter tidssteg
sampleTime = 0.05;
vizRate = rateControl(1/sampleTime);


%Bestemmer hvor stor roboten skal være 
frameSize = robotBase.WheelBase*2;

%Kjører til vi er kommet nært nok mål 
while( distanceToGoal > goalRadius )        
   
    %Robot input from controller
    [v, omega] = H2rObot_controller(robotCurrentPose);
   
    %Robot hastighet 
    vel = derivative(robotBase, robotCurrentPose, [v omega]);
    
    %Oppdaterer pose 
    robotCurrentPose = robotCurrentPose + vel*sampleTime 
    
  
    %Oppdeterer avstand til målpunkt
    distanceToGoal = norm(robotCurrentPose(1:2) - robotMaalpos(:))
 
    hold off
   
    %Simulerer bevegelsen av roboten
    lp2.plot()
    hold all
    plot(path_plot(:,1), path_plot(:,2),'k--d')
   
    %Plotter banen til roboten
    plotTrVec = [robotCurrentPose(1:2); 0]*10;
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize)
    light;
    xlim([0 170])
    ylim([0 120])
    
    waitfor(vizRate);
end


