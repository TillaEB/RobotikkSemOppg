%% ELE306 Robotikk  semesteroppgave  
%% Gruppe 2

clc;
clear;

%% Hvordan kjøre scriptet: 
% 1) launch roboten i gazebo 
% 2) kjør pathtool i command window første gang du åpner matlab, flytt toolboksene til toppen
% 3) sørg for å ha kjopesenter_skript_nyesr oppe i en annen fane 
% 4) kjør scriptet 
% 
% Om du får error på odom.Pose.Pose.Position skriv rosshutdown i command window,
% og kjør scriptet på nytt. 


%% Legger inn egenskaper til H2rObot 
maksStyresvinkel=deg2rad(40);
breddeRobot=0.70;
wheelbase=0.60;


%% Kart
% Henter kartet over kjøpesenteret og punktene den skal innom 
kjopesenter_skript_nyesr;

% Lattice planner 
lp = Lattice(kjopesenter,'grid',3,'root',[8,3,0],'inflate',1,'cost',[1,10,10] ) 
lp.plan('cost',[1,10,10]) % koster mer å svinge enn å kjøre rett frem
lp2 = Lattice(kjopesenter,'grid',3,'root',[8,3,0],'inflate',1,'cost',[1,10,10]) 

save lp
save lp2
%lp.plot(

%Henter lagret lattice planner 
load lp
lp.plot()


%% Simulerer i ros
rosshutdown % Avslutter gammel ros kobling som ikke har blitt lukket ordenig ved tidligere kjøringer 
rosinit % Starter ros 

global odom 
 
sub_odom = rossubscriber("/robot_base_velocity_controller/odom",@odom_callback)

% Mobil Base kontroller
[pub_vel,msg_vel] = rospublisher('/robot_base_velocity_controller/cmd_vel','geometry_msgs/Twist');

% Arm controller
pub = rospublisher('arm_controller','trajectory_msgs/JointTrajectory');

msg = rosmessage('trajectory_msgs/JointTrajectory');

%Stopper robot
msg_vel.Linear.X = 0.0;
msg_vel.Angular.Z = 0.0;
send(pub_vel,msg_vel)

%Finner rute fra start posisjon til slutt posisjon 
p=lp.query([8 3 0], [62 21 0]) % Velger P2 og P3 fordi den vet vi skal gå

%Henter ut x og y kordinatene
p1 = p(:,1);
p2 = p(:,2);

%Setter x og y kordinantene sammen til en matrise  
path = [p1,p2]/10;
path_plot = path*10;

lp.plot(p)

%% Kontroller, med PurePursiut og bicyclemodel
% Start og slutt posisjon for kontroller
robotStartPos = path(1,:);
robotMaalpos = path(end,:);
initialOrientation = 0;

% Start Pose til basen
robotCurrentPose = [robotStartPos initialOrientation]';

% Kinematisk modell for mobilbase 
robotBase = bicycleKinematics("WheelBase",wheelbase,"MaxSteeringAngle",maksStyresvinkel,"VehicleInputs","VehicleSpeedSteeringAngle"); 
%robotBase = differentialDriveKinematics("TrackWidth", 0.43, "VehicleInputs", "VehicleSpeedHeadingRate");

% Kontroller med PurePursuit:
H2rObot_controller = controllerPurePursuit;
H2rObot_controller.Waypoints = path; %Gir kontrolleren banen den skal følge 

H2rObot_controller.DesiredLinearVelocity = 0.5; % Setter ønsket lineær hastighet, Krav på 0.5m/s
H2rObot_controller.MaxAngularVelocity = 2.0; % Setter svinghastighet / vinkelhastighet
H2rObot_controller.LookaheadDistance = 0.5; % Setter hvor langt frem kontrolleren skal se for å beregne Pure Pursuit

goalRadius = 0.1; % Hvor nært mål vi ønsker å komme 
distanceToGoal = norm(robotStartPos - robotMaalpos); % Setter avstand til mål
frameSize = robotBase.WheelBase*5; %Bestemmer hvor stor roboten skal være 

% Simulerer bevegelsen av roboten
figure
lp2.plot()
hold all
plot(path_plot(:,1), path_plot(:,2),'k--d')

% Plotter banen til roboten
plotTrVec = [robotCurrentPose(1:2); 0]*10;
plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize) % Må ha groundvehicle.stl ikke robot_base.stl
light;
xlim([0 170])
ylim([-20 80])

%% Simuleringen
% Setter tidssteg
sampleTime = 0.10;
vizRate = rateControl(1/sampleTime);


% Kjører til vi har kommet nær nok mål 
while( distanceToGoal > goalRadius ) 
    odom.Pose.Pose.Position
   
    % Robot input fra Kontroller
    [v, omega] = H2rObot_controller(robotCurrentPose);
    
    % Fart til  basen
	msg_vel.Linear.X = v;
	msg_vel.Angular.Z = omega;
    
    % Sender meldingen om basens hastighet og retning
    send(pub_vel,msg_vel)
   
    % Henter basens rotasjon og posisjon:
    % Posisjon
    odomMsg = receive(sub_odom,3);
    pose = odomMsg.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
    % Rotasjon
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
   
    robotCurrentPose =  [x;y;angles(1)]  %robotCurrentPose + vel*sampleTime % Oppdaterer pose
    distanceToGoal = norm(robotCurrentPose(1:2) - robotMaalpos(:)) % Oppdaterer avstand tl målet 
    
    % Oppdaterer plottet 
    hold off
    
    % Simulerer bevegelsen av roboten
    lp2.plot()
    hold all
    plot(path_plot(:,1), path_plot(:,2),'k--d')
   
    % Plotter banen til roboten
    plotTrVec = [robotCurrentPose(1:2); 0]*10;
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize) % Må ha groundvehicle.stl ikke robot_base.stl
    light;
    xlim([0 170])
    ylim([-20 80])

        waitfor(vizRate);
end

% Stopper roboten
msg_vel.Linear.X = 0.0;
msg_vel.Angular.Z = 0.0;

send(pub_vel,msg_vel) % Sender melding 

     
rosshutdown % Stopper ros

function odom_callback(src,msg)
    global odom
    odom = msg; 
end
