%% Vanne plante 
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
%lp.plot()

%Henter lagret lattice planner 
load lp
lp.plot()


%% Lager arm:
import ETS3.*

% Lengder på ledd 
B= 200 + 600;    % 80 cm 
L1 = 400;        % 40 cm 
L2 = 400;        % 40 cm 
L3 = 400;        % 40 cm 
L4 = 400;        % 40 cm  
L5 = 250;        % 25 cm 

% DH parametere
j1 = Revolute('d',L1,'a',0,'alpha', -pi/2, 'offset',0)
j2 = Revolute('d',0,'a',-L2,'alpha', 0, 'offset',pi/2)
j3 = Revolute('d',0,'a',-L3,'alpha', 0, 'offset',0)
j4 = Revolute('d',0,'a',-L4,'alpha', 0, 'offset',0)
j5 = Revolute('d',0,'a',-L5,'alpha', 0, 'offset',0)

% Setter sammen DH parametere for å lage roboten 
H2rObot_arm = SerialLink([j1 j2 j3 j4 j5],'name','H2rObot')

%% Arm posisjoner: 
% Armen er 160 over bakken når den er i disse posisjonene 
q_se_etter_plante_venstre = [0,0,pi/2,0,0]; % Armvinkler for å se etter plante venstre
%q_se_etter_plante_hoyre = [-pi,0,pi/2,0,0]; % Armvinkler for å se etter plante høyre 

q_kjore = [-pi/2,0,pi/2,pi/2,0]; % Armvinkler for kjøre posisjon 

% Transformasjonsmatrise for start posisjonen fra forwad kinematikk 
T_H2rObot_O = H2rObot_arm.fkine([0,0,0,0,0]) 

% Transformasonsmatriser for base til end-effector i 
% se etter plante venstre posisjon og se etter plante høyre posisjon 
BTEV = H2rObot_arm.fkine(q_se_etter_plante_venstre)
%BTEH = H2rObot_arm.fkine(q_se_etter_plante_hoyre)

% Kameraet sitter 6 cm langs x-aksen til end-effector 
ETK = transl(0,0,60)

% Kameraet ser en plante 100 mm (10 cm) nedover (y-retning) og 70 mm (7 cm)
% til høyre (z-retning)
KTPhoyv = transl(0,-100,-70)  %% Planten er 150 cm 

% Kameraet ser en plante 1282 mm (128,2 cm) (maks) nedover (y-retning) 
% og 80 mm (8 cm) innover mot basen (x-retning) 
KTPlavv = transl(-80,-1282,0) % Planten er 20 cm  

% Kamera ser en plante 600 mm (60 cm) nedover (y-retning) og 60 mm (6 cm) 
% til venstre (z-retning)
%KTPhoyh  = transl(0,-600,60) % Planten er 100 cm

% Kamera ser en plante 1100 mm (110 cm) nedover (y-retning), 20 mm (2 cm) 
% til høyre (z-retning), og 80 mm (8 cm) innover mot basen (x-retning)
%KTPlavh = transl(-80,-1100,-20) % planten er 50 cm 

% Transformasjonsmatrise for base til vanne plante høy venstre 
BTVP_Hoyv = BTEV.T * KTPhoyv * ETK

% Transformasjonsmatrise for base til vanne plante lav venstre 
BTVP_lavv = BTEV.T * KTPlavv * ETK 

% Transformasjonsmatrise for base til vanne plante høy høyre 
%BTVP_Hoyh = BTEH.T * KTPhoyh * ETK

% Transformasjonsmatrise for base til vanne plante lav høyre 
%BTVP_lavh = BTEH.T * KTPlavh * ETK 
% 
% Fordi vi har en underaktuert robotarm har vi valg å ignorere rx, ry og rz
% fordi vi ikke har behov for de SIKKER?

% Armvinkler for start posisjon fra invers kinematikk på
% transformasjonsmatrisen 
q0 = H2rObot_arm.ikine(T_H2rObot_O,'mask',[1 1 1 0 0 0])

% Armvinkler for at end-effector er i samme posisjon som planten, 
% høy venstre 
q1 = H2rObot_arm.ikine(BTVP_Hoyv,'q0', q0,'mask',[1 1 1 0 0 0])

% Armvinkler for at end-effector er i samme posisjon som planten, 
% lav venstre 
q2 = H2rObot_arm.ikine(BTVP_lavv,'q0', q0,'mask',[1 1 1 0 0 0])

% Armvinkler for at end-effector er i samme posisjon som planten, 
% høy høyre 
%q3 = H2rObot_arm.ikine(BTVP_Hoyh,'q0', q0,'mask',[1 1 1 0 0 0])

% Armvinkler for at end-effector er i samme posisjon som planten, 
% lav høyre 
%q4 = H2rObot_arm.ikine(BTVP_lavh,'q0', q0,'mask',[1 1 1 0 0 0])


%% Lager bane:
t = [0:0.05:2]'; % tidssteg
  
m0 = mtraj(@tpoly, q0,q_kjore,t);                                % Bane fra rett arm til kjøreposisjon
m1 = mtraj(@tpoly, q_kjore, q_se_etter_plante_venstre, t);       % Bane fra kjøreposisjon til se etter plante venstre posisjon
m2 = mtraj(@tpoly, q_se_etter_plante_venstre, q1, t);            % Bane fra se etter plante venstre posisjon til vanne plante høy venstre posisjon
m3 = mtraj(@tpoly, q1, q_kjore, t);                              % Bane fra vanne plante høy venstre posisjon til kjøreposisjon 
m4 = mtraj(@tpoly, q_kjore, q_se_etter_plante_venstre, t);       % Bane fra kjøreposisjon til se etter plante venstre posisjon
m5 = mtraj(@tpoly, q_se_etter_plante_venstre, q2, t);            % Bane fra se etter plante venstre posisjon til vanne plante lav venstre posisjon
m6 = mtraj(@tpoly, q2, q_kjore, t);                              % Banse fra vanne plante lav venstre posisjon til kjøreposisjon 
% m7 = mtraj(@tpoly, q_kjore, q_se_etter_plante_hoyre, t);         % Bane fra kjøreposisjon til se etter plante høyre posisjon 
% m8 = mtraj(@tpoly, q_se_etter_plante_hoyre, q3, t);              % Bane fra se etter plate høyre posisjon til vanne plante høy høyre posisjon 
% m9 = mtraj(@tpoly, q3, q_kjore, t);                              % Bane fra vanne plante høy høyre posisjon til kjøreposisjon 
% m10 = mtraj(@tpoly, q_kjore, q_se_etter_plante_hoyre, t);        % Bane fra kjøreposisjon til se etter plante høyre posisjon 
% m11 = mtraj(@tpoly, q_se_etter_plante_hoyre, q4, t);             % Bane fra se etter plante høyre posisjon til vanne plante lav høyre posisjon 
% m12 = mtraj(@tpoly, q4, q_kjore, t);   


%% Simulerer i ros
rosshutdown % Avslutter gammel ros kobling som ikke har blitt lukket ordenig ved tidligere kjøringer 
rosinit % Starter ros 
global odom 
 
sub_odom = rossubscriber("/robot_base_velocity_controller/odom",@odom_callback)

% Mobil Base kontroller
[pub_vel,msg_vel] = rospublisher ('/arm_controller/command','trajectory_msgs/JointTrajectory');


% Arm kontroller
[pub_q1,msg_q1] = rosmessage('trajectory_msgs/JointTrajectoryPoint');
[pub_q2,msg_q2] = rosmessage('trajectory_msgs/JointTrajectoryPoint');
[pub_q3,msg_q3] = rosmessage('trajectory_msgs/JointTrajectoryPoint');
[pub_q4,msg_q4] = rosmessage('trajectory_msgs/JointTrajectoryPoint');
[pub_q5,msg_q5] = rosmessage('trajectory_msgs/JointTrajectoryPoint');

q_temp = [0,0,0,0,0];

msg = rosmessage(pub_joints);
msg.JointNames = {'arm_base_joint' 'shoulder_joint' 'bottom_wrist_joint' 'elbow_joint' 'top_wrist_joint' };
msg.Points = [[pub_q1,msg_q1],[pub_q2,msg_q2],[pub_q3,msg_q3],[pub_q4,msg_q4],[pub_q5,msg_q5]]

%Antall bevegelser i hver bane
q0_moves = size(m0,1);
q1_moves = size(m1,1);
q2_moves = size(m2,1);
q3_moves = size(m3,1);
q4_moves = size(m4,1);
q5_moves = size(m5,1);
q6_moves = size(m6,1);


%Initialiserer Indexer for hver bane
index0 = 1;
index1 = 1;
index2 = 1;
index3 = 1;
index4 = 1;
index5 = 1;
index6 = 1;

%Stopper robot
msg_vel.Linear.X = 0.0;
msg_vel.Angular.Z = 0.0;
send(pub_vel,msg_vel)

%Setter arm i start posisjon:
rate = robotics.Rate(10);
kjor = true;
while kjor
    odom.Pose.Pose.Position
    %Stepper igjnennom banen for armen
    %Fra Home til eple
    if index0 <= q0_moves
        q_temp = m0(index0,:);
        index0 = index0 +1;  
    else
        kjor = false;
    end
    %Setter vinkel verdier for hvert ledd inn i rett ut melding
    msg_q1.Data = q_temp(1);
    msg_q2.Data = q_temp(2);
    msg_q3.Data = q_temp(3);
    msg_q4.Data = q_temp(4);
    msg_q5.Data = q_temp(5);
    %Sender meldingene om vinklene ut på ros-netverket
    send(pub_q1,msg_q1)
    send(pub_q2,msg_q2)
    send(pub_q3,msg_q3)
    send(pub_q4,msg_q4) 
    send(pub_q5,msg_q5)
     
    waitfor(rate);
end

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

%frameSize = robotBase.WheelBase*5; %Bestemmer hvor stor roboten skal være

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

%Setter arm i start posisjon:
rate = robotics.Rate(10);
kjor = true;
while kjor
    %Stepper igjnennom banen for armen
    %Fra Home til eple
        odom.Pose.Pose.Position

    if index1 <= q1_moves
        q_temp = m1(index1,:);
        index1 = index0 +1;    
    elseif index2 <= q2_moves
        if index2 == 1 
            pause(2)
        end
        q_temp = m2(index2,:);
        index2 = index2 +1;
     elseif index3 <= q3_moves
        if index3 == 1 
            pause(2)
        end
        q_temp = m3(index3,:);
        index3 = index3 +1;
     elseif index4 <= q4_moves
        if index4 == 1 
            pause(2)
        end
        q_temp = m4(index4,:);
        index4 = index4 +1;
     elseif index5 <= q5_moves
        if index5 == 1 
            pause(2)
        end
        q_temp = m5(index2,:);
        index5 = index5 +1;
     elseif index6 <= q6_moves
        if index6 == 1 
            pause(2)
        end
        q_temp = m6(index6,:);
        index6 = index6 +1;
       
       
    else
        index0 = 1;
        index1 = 1;
        index2 = 1;
        index3 = 1;
        index4 = 1;
        index5 = 1;
        index6 = 1;
        kjor = false;
    end
        %Setter vinkel verdier for hvert ledd inn i rett ut melding
        msg_q1.Data = q_temp(1);
        msg_q2.Data = q_temp(2);
        msg_q3.Data = q_temp(3);
        msg_q4.Data = q_temp(4);
        msg_q5.Data = q_temp(5);
        %Sender meldingene om vinklene ut på ros-netverket
        send(pub_q1,msg_q1)
        send(pub_q2,msg_q2)
        send(pub_q3,msg_q3)
        send(pub_q4,msg_q4) 
        send(pub_q5,msg_q5)
     
    waitfor(rate);
 
end


rosshutdown % Stopper ros

function odom_callback(src,msg)
    global odom
    odom = msg; 
end



% %% Vanne plante 
% clc;
% clear;
% 
% %% Hvordan kjøre scriptet: 
% % 1) launch roboten i gazebo 
% % 2) kjør pathtool i command window første gang du åpner matlab, flytt toolboksene til toppen
% % 3) sørg for å ha kjopesenter_skript_nyesr oppe i en annen fane 
% % 4) kjør scriptet 
% % 
% % Om du får error på odom.Pose.Pose.Position skriv rosshutdown i command window,
% % og kjør scriptet på nytt. 
% 
% 
% %% Legger inn egenskaper til H2rObot 
% maksStyresvinkel=deg2rad(40);
% breddeRobot=0.70;
% wheelbase=0.60;
% 
% 
% %% Kart
% % Henter kartet over kjøpesenteret og punktene den skal innom 
% kjopesenter_skript_nyesr;
% 
% % Lattice planner 
% lp = Lattice(kjopesenter,'grid',3,'root',[8,3,0],'inflate',1,'cost',[1,10,10] ) 
% lp.plan('cost',[1,10,10]) % koster mer å svinge enn å kjøre rett frem
% lp2 = Lattice(kjopesenter,'grid',3,'root',[8,3,0],'inflate',1,'cost',[1,10,10]) 
% 
% save lp
% save lp2
% %lp.plot()
% 
% %Henter lagret lattice planner 
% load lp
% lp.plot()
% 
% % Lager arm 
% import ETS3.*
%  
% % Lengder på ledd 
% B=0.20+0.60; 
% L1=0.40; 
% L2=0.40; 
% L3=0.40; 
% L4=0.40; 
% L5=0.25;
% 
% % DH parametere 
% L(1)=Link('revolute','d',0,'a',40,'alpha',-pi/2)
% L(2)=Link('revolute','d',0,'a',40,'alpha',0)
% L(3)=Link('revolute','d',0,'a',40,'alpha',0)
% L(4)=Link('revolute','d',0,'a',40,'alpha',0)
% L(5)=Link('revolute','d',0,'a',25,'alpha',0)
% 
% % Setter sammen DH parametere for å lage roboten 
% H2rObot_arm = SerialLink(L,'name','H2rObot')
% 
% %% Arm kinematikk 
% T_robot_0 = H2rObot_arm.fkine([0,0,0,0,0]) % Startposisjons transformasjonsmatrise fra forwad kinematics 
% q_HV = [0, 0.1745, 0.5236, 0, 1.5708] % Vinkler for å vanne høy plante venstre
% bTe= H2rObot_arm.fkine(q_HV) % Transformasjonsmatrise for base til end-effektor
% 
% 
% q_start = [-(pi/2),0,1.5708,0,1.5708]; % Vinkler til armen for kjøreposisjonen 
% 
% q0 = H2rObot_arm.ikine(T_robot_0, 'mask', [1 1 1 0 0 1]) % Startvinkler fra inverse kinematics 
% 
% q1 = H2rObot_arm.ikine(bTe,'q0', q0(end,:), 'mask', [1 1 1 0 0 1]) % Finner vinkler for at endefektor skal være i samme posisjon som plante
% 
% 
% %% Bane planlegging
% t = [0:0.05:2]'; % tidssteg
% m0 = mtraj(@tpoly, q0,q_start,t); % Bane fra rett arm til startposisjon
% m1 = mtraj(@tpoly, q_start, q_HV, t); % Bane fra start til plante
% m2 = mtraj(@tpoly, q1, q_start, t); % Bane fra plante til startposisjon
% 
% 
% %% Simulerer i ros
% rosshutdown % Avslutter gammel ros kobling som ikke har blitt lukket ordenig ved tidligere kjøringer 
% rosinit % Starter ros 
% global odom 
%  
% sub_odom = rossubscriber("/robot_base_velocity_controller/odom",@odom_callback)
% 
% % Mobil Base kontroller
% [pub_vel,msg_vel] = rospublisher('/robot_base_velocity_controller/cmd_vel','geometry_msgs/Twist');
% 
% % Arm kontroller
% [pub_q1,msg_q1] = rospublisher('/arm_controller/arm_base_joint/command','std_msgs/Float64');
% [pub_q2,msg_q2] = rospublisher('/arm_controller/shoulder_joint/command','std_msgs/Float64');
% [pub_q3,msg_q3] = rospublisher('/arm_controller/bottom_wrist_joint/command','std_msgs/Float64');
% [pub_q4,msg_q4] = rospublisher('/arm_controller/elbow_joint/command','std_msgs/Float64');
% [pub_q5,msg_q5] = rospublisher('/arm_controller/top_wrist_joint/command','std_msgs/Float64');
% 
% q_temp = [0,0,0,0,0];
% 
% % Antall bevegelser i hver bane
% q0_moves = size(m0,1);
% q1_moves = size(m1,1);
% q2_moves = size(m2,1);
% 
% % Indexer for hver bane
% index0 = 1;
% index1 = 1;
% index2 = 1;
% 
% % Stopper robot
% msg_vel.Linear.X = 0.0;
% msg_vel.Angular.Z = 0.0;
% send(pub_vel,msg_vel)
% 
% 
% % Setter arm i start posisjon:
% rate = robotics.Rate(10);
% kjor = true;
% 
% while kjor
%     odom.Pose.Pose.Position
%     % Går gjennom banen for armen 
%     
%     % Start til plante
%     if index0 <= q0_moves
%         q_temp = m0(index0,:);
%         index0 = index0 +1;  
%     else
%         kjor = false;
%     end
%     
%     % Vinkelverdier for hvert ledd 
%     msg_q1.Data = q_temp(1);
%     msg_q2.Data = q_temp(2);
%     msg_q3.Data = q_temp(3);
%     msg_q4.Data = q_temp(4);
%     msg_q5.Data = q_temp(5);   
%     
%     % Sender meldingene om vinklene ut til ros
%     send(pub_q1,msg_q1)
%     send(pub_q2,msg_q2)
%     send(pub_q3,msg_q3)
%     send(pub_q4,msg_q4) 
%     send(pub_q5,msg_q5)
%      
%     waitfor(rate);
% end
% 
% 
% %Finner rute fra start posisjon til slutt posisjon 
% p=lp.query([8 3 0], [62 21 0]) % Velger P2 og P3 fordi den vet vi skal gå
% 
% %Henter ut x og y kordinatene
% p1 = p(:,1);
% p2 = p(:,2);
% 
% %Setter x og y kordinantene sammen til en matrise  
% path = [p1,p2]/10;
% path_plot = path*10;
% 
% lp.plot(p)
% 
% %% Kontroller, med PurePursiut og bicyclemodel
% 
% % Start og slutt posisjon for kontroller
% robotStartPos = path(1,:);
% robotMaalpos = path(end,:);
% initialOrientation = 0;
% 
% % Start Pose til basen
% robotCurrentPose = [robotStartPos initialOrientation]';
% 
% % Kinematisk modell for mobilbase 
% robotBase = bicycleKinematics("WheelBase",wheelbase,"MaxSteeringAngle",maksStyresvinkel,"VehicleInputs","VehicleSpeedSteeringAngle"); 
% %robotBase = differentialDriveKinematics("TrackWidth", 0.43, "VehicleInputs", "VehicleSpeedHeadingRate");
% 
% 
% % Kontroller med PurePursuit:
% H2rObot_controller = controllerPurePursuit;
% H2rObot_controller.Waypoints = path; %Gir kontrolleren banen den skal følge 
% 
% H2rObot_controller.DesiredLinearVelocity = 0.5; % Setter ønsket lineær hastighet, Krav på 0.5m/s
% H2rObot_controller.MaxAngularVelocity = 2.0; % Setter svinghastighet / vinkelhastighet
% H2rObot_controller.LookaheadDistance = 0.5; % Setter hvor langt frem kontrolleren skal se for å beregne Pure Pursuit
% 
% goalRadius = 0.1; % Hvor nært mål vi ønsker å komme 
% distanceToGoal = norm(robotStartPos - robotMaalpos); % Setter avstand til mål
% frameSize = robotBase.WheelBase*5; %Bestemmer hvor stor roboten skal være 
% 
% 
% % Simulerer bevegelsen av roboten
% figure
% lp2.plot()
% hold all
% plot(path_plot(:,1), path_plot(:,2),'k--d')
% 
% % Plotter banen til roboten
% plotTrVec = [robotCurrentPose(1:2); 0]*10;
% plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
% plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize) % Må ha groundvehicle.stl ikke robot_base.stl
% light;
% xlim([0 170])
% ylim([-20 80])
% 
% 
% %% Simuleringen
% 
% % Setter tidssteg
% sampleTime = 0.10;
% vizRate = rateControl(1/sampleTime);
% 
% %frameSize = robotBase.WheelBase*5; %Bestemmer hvor stor roboten skal være
% 
% % Kjører til vi har kommet nær nok mål 
% while( distanceToGoal > goalRadius ) 
%     odom.Pose.Pose.Position
%    
%     % Robot input fra Kontroller
%     [v, omega] = H2rObot_controller(robotCurrentPose);
%     
%     % Fart til  basen
% 	msg_vel.Linear.X = v;
% 	msg_vel.Angular.Z = omega;
%     
%     
%     % Sender meldingen om basens hastighet og retning
%     send(pub_vel,msg_vel)
%    
%     % Henter basens rotasjon og posisjon:
%     % Posisjon
%     odomMsg = receive(sub_odom,3);
%     pose = odomMsg.Pose.Pose;
%     x = pose.Position.X;
%     y = pose.Position.Y;
%     % Rotasjon
%     quat = pose.Orientation;
%     angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%    
%     robotCurrentPose =  [x;y;angles(1)]  %robotCurrentPose + vel*sampleTime % Oppdaterer pose
%     distanceToGoal = norm(robotCurrentPose(1:2) - robotMaalpos(:)) % Oppdaterer avstand tl målet 
%     
%     % Oppdaterer plottet 
%     hold off
%     
%     % Simulerer bevegelsen av roboten
%     lp2.plot()
%     hold all
%     plot(path_plot(:,1), path_plot(:,2),'k--d')
%    
%     % Plotter banen til roboten
%     plotTrVec = [robotCurrentPose(1:2); 0]*10;
%     plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
%     plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize) % Må ha groundvehicle.stl ikke robot_base.stl
%     light;
%     xlim([0 170])
%     ylim([-20 80])
% 
%         waitfor(vizRate);
% end
% 
% % Stopper roboten
% msg_vel.Linear.X = 0.0;
% msg_vel.Angular.Z = 0.0;
% 
% send(pub_vel,msg_vel) % Sender melding 
% 
% 
% % %% Vanner plante 
% % % Hvor mange ganger i sekundet komandoene skal bli sendt
% rate = robotics.Rate(10);
% kjor = true;
% 
% while kjor
% odom.Pose.Pose.Position
%     % Går gjennom banen for armen 
%     
%     %Fra start til plante
%     if index1 <= q1_moves
%         q_temp = m1(index1,:);
%         index1 = index1 +1;  
%         
%     %Fra plante til kjøre
%     elseif index2 <= q2_moves
%         if index2 == 1 
%             pause(2)
%         end
%         q_temp = m2(index2,:);
%         index2 =  index2 +1;  
%     
%     else
%         index1 = 1;
%         index2 = 1;
%         kjor = false;
%     end
%     
%     % Vinkelverdier for hvert ledd 
%     msg_q1.Data = q_temp(1);
%     msg_q2.Data = q_temp(2);
%     msg_q3.Data = q_temp(3);
%     msg_q4.Data = q_temp(4);
%     msg_q5.Data = q_temp(5);   
%     
%     % Sender meldingene om vinklene ut til ros
%     send(pub_q1,msg_q1)
%     send(pub_q2,msg_q2)
%     send(pub_q3,msg_q3)
%     send(pub_q4,msg_q4) 
%     send(pub_q5,msg_q5)
%      
%     waitfor(rate);
% end
% 
% rosshutdown % Stopper ros
% 
% function odom_callback(src,msg)
%     global odom
%     odom = msg; 
% end
% 
% 
% 
% 
% 
% 
% 
% 
% 
% % %% Vanne plante 
% % clc;
% % clear;
% % 
% % %% Hvordan kjøre scriptet: 
% % % 1) launch roboten i gazebo 
% % % 2) kjør pathtool i command window første gang du åpner matlab, flytt toolboksene til toppen
% % % 3) sørg for å ha kjopesenter_skript_nyesr oppe i en annen fane 
% % % 4) kjør scriptet 
% % % 
% % % Om du får error på odom.Pose.Pose.Position skriv rosshutdown i command window,
% % % og kjør scriptet på nytt. 
% % 
% % 
% % %% Legger inn egenskaper til H2rObot 
% % maksStyresvinkel=deg2rad(40);
% % breddeRobot=0.70;
% % wheelbase=0.60;
% % 
% % 
% % %% Kart
% % % Henter kartet over kjøpesenteret og punktene den skal innom 
% % kjopesenter_skript_nyesr;
% % 
% % % Lattice planner 
% % lp = Lattice(kjopesenter,'grid',3,'root',[80,30,0],'inflate',1,'cost',[1,10,10]) 
% % lp.plan('cost',[1,10,10]) % koster mer å svinge enn å kjøre rett frem
% % lp2 = Lattice(kjopesenter,'grid',3,'root',[80,30,0],'inflate',1,'cost',[1,10,10]) 
% % 
% % save lp
% % save lp2
% % lp.plot()
% % 
% % %Henter lagret lattice planner 
% % load lp
% % 
% % 
% % %% Lager arm 
% % import ETS3.*
% % 
% % % Lengder på ledd 
% % B=0.20+0.60; 
% % L1=0.40; 
% % L2=0.40; 
% % L3=0.40; 
% % L4=0.40; 
% % L5=0.25;
% % 
% % % DH parametere 
% % L(1)=Link('revolute','d',0,'a',40,'alpha',-pi/2)
% % L(2)=Link('revolute','d',0,'a',40,'alpha',0)
% % L(3)=Link('revolute','d',0,'a',40,'alpha',0)
% % L(4)=Link('revolute','d',0,'a',40,'alpha',0)
% % L(5)=Link('revolute','d',0,'a',25,'alpha',0)
% % 
% % % Setter sammen DH parametere for å lage roboten 
% % H2rObot_arm = SerialLink(L,'name','H2rObot')
% % 
% % %% Arm kinematikk 
% % T_robot_0 = H2rObot_arm.fkine([0,0,0,0,0]) % Startposisjons transformasjonsmatrise fra forwad kinematics 
% % q_HV = [0, 0.1745, 0.5236, 0, 1.5708] % Vinkler for å vanne høy plante venstre
% % bTe= H2rObot_arm.fkine(q_HV) % Transformasjonsmatrise for base til end-effektor
% % 
% % 
% % q_start = [-(pi/2),0,1.5708,0,1.5708]; % Vinkler til armen for kjøreposisjonen 
% % 
% % q0 = H2rObot_arm.ikine(T_robot_0, 'mask', [1 1 1 0 0 1]) % Startvinkler fra inverse kinematics 
% % 
% % q1 = H2rObot_arm.ikine(bTe,'q0', q0(end,:), 'mask', [1 1 1 0 0 1]) % Finner vinkler for at endefektor skal være i samme posisjon som plante
% % 
% % 
% % %% Bane planlegging
% % t = [0:0.05:2]'; % tidssteg
% % m0 = mtraj(@tpoly, q0,q_start,t); % Bane fra rett arm til startposisjon
% % m1 = mtraj(@tpoly, q_start, q_HV, t); % Bane fra start til plante
% % m2 = mtraj(@tpoly, q1, q_start, t); % Bane fra plante til startposisjon
% % 
% % 
% % %% Simulerer i ros
% % rosshutdown % Avslutter gammel ros kobling som ikke har blitt lukket ordenig ved tidligere kjøringer 
% % rosinit % Starter ros 
% % global odom 
% %  
% % sub_odom = rossubscriber("/robot_base_velocity_controller/odom",@odom_callback)
% % 
% % % Mobil Base kontroller
% % [pub_vel,msg_vel] = rospublisher('/robot_base_velocity_controller/cmd_vel','geometry_msgs/Twist');
% % 
% % % Arm kontroller
% % [pub_q1,msg_q1] = rospublisher('/arm_controller/arm_base_joint/command','std_msgs/Float64');
% % [pub_q2,msg_q2] = rospublisher('/arm_controller/shoulder_joint/command','std_msgs/Float64');
% % [pub_q3,msg_q3] = rospublisher('/arm_controller/bottom_wrist_joint/command','std_msgs/Float64');
% % [pub_q4,msg_q4] = rospublisher('/arm_controller/elbow_joint/command','std_msgs/Float64');
% % [pub_q5,msg_q5] = rospublisher('/arm_controller/top_wrist_joint/command','std_msgs/Float64');
% % 
% % q_temp = [0,0,0,0,0];
% % 
% % % Antall bevegelser i hver bane
% % q0_moves = size(m0,1);
% % q1_moves = size(m1,1);
% % q2_moves = size(m2,1);
% % 
% % % Indexer for hver bane
% % index0 = 1;
% % index1 = 1;
% % index2 = 1;
% % 
% % % Stopper robot
% % msg_vel.Linear.X = 0.0;
% % msg_vel.Angular.Z = 0.0;
% % send(pub_vel,msg_vel)
% % 
% % 
% % % Setter arm i start posisjon:
% % rate = robotics.Rate(10);
% % kjor = true;
% % 
% % while kjor
% %     odom.Pose.Pose.Position
% %     % Går gjennom banen for armen 
% %     
% %     % Start til plante
% %     if index0 <= q0_moves
% %         q_temp = m0(index0,:);
% %         index0 = index0 +1;  
% %     else
% %         kjor = false;
% %     end
% %     
% %     % Vinkelverdier for hvert ledd 
% %     msg_q1.Data = q_temp(1);
% %     msg_q2.Data = q_temp(2);
% %     msg_q3.Data = q_temp(3);
% %     msg_q4.Data = q_temp(4);
% %     msg_q5.Data = q_temp(5);   
% %     
% %     % Sender meldingene om vinklene ut til ros
% %     send(pub_q1,msg_q1)
% %     send(pub_q2,msg_q2)
% %     send(pub_q3,msg_q3)
% %     send(pub_q4,msg_q4) 
% %     send(pub_q5,msg_q5)
% %      
% %     waitfor(rate);
% % end
% % 
% % 
% % % Finner rute fra start posisjon til slutt posisjon 
% % p=lp.query(P2,P3) % Velger P2 og P3 fordi den vet vi skal gå
% % 
% % Henter ut x og y kordinatene
% p1 = p(:,1);
% p2 = p(:,2);
% 
% % Setter x og y kordinantene sammen til en matrise  
% path = [p1,p2]/10;
% path_plot = path*10;
% 
% lp.plot(p)
% 
% %% Kontroller, med PurePursiut og bicyclemodel
% 
% % Start og slutt posisjon for kontroller
% robotStartPos = path(1,:);
% robotMaalpos = path(end,:);
% initialOrientation = 0;
% 
% % Start Pose til basen
% robotCurrentPose = [robotStartPos initialOrientation]';
% 
% % Kinematisk modell for mobilbase 
% robotBase = bicycleKinematics("WheelBase",wheelbase,"MaxSteeringAngle",maksStyresvinkel,"VehicleInputs","VehicleSpeedSteeringAngle"); 
% 
% % Kontroller med PurePursuit:
% H2rObot_controller = controllerPurePursuit;
% H2rObot_controller.Waypoints = path; %Gir kontrolleren banen den skal følge 
% 
% H2rObot_controller.DesiredLinearVelocity = 0.5; % Setter ønsket lineær hastighet, Krav på 0.5m/s
% H2rObot_controller.MaxAngularVelocity = 2; % Setter svinghastighet / vinkelhastighet
% H2rObot_controller.LookaheadDistance = 0.5; % Setter hvor langt frem kontrolleren skal se for å beregne Pure Pursuit
% 
% goalRadius = 0.1; % Hvor nært mål vi ønsker å komme 
% distanceToGoal = norm(robotStartPos - robotMaalpos); % Setter avstand til mål
% frameSize = robotBase.WheelBase*5; %Bestemmer hvor stor roboten skal være 
% 
% 
% % Simulerer bevegelsen av roboten
% figure
% lp2.plot()
% hold all
% plot(path_plot(:,1), path_plot(:,2),'k--d')
% 
% % Plotter banen til roboten
% plotTrVec = [robotCurrentPose(1:2); 0]*10;
% plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
% plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize) % Må ha groundvehicle.stl ikke robot_base.stl
% light;
% xlim([0 170])
% ylim([0 120])
% 
% 
% %% Simuleringen
% 
% % Setter tidssteg
% sampleTime = 0.05;
% vizRate = rateControl(1/sampleTime);
% 
% % Kjører til vi har kommet nær nok mål 
% while( distanceToGoal > goalRadius ) 
%     odom.Pose.Pose.Position
%    
%     % Robot input fra Kontroller
%     [v, omega] = H2rObot_controller(robotCurrentPose);
%     
%     % Fart til  basen
% 	msg_vel.Linear.X = v;
% 	msg_vel.Angular.Z = omega;
%     
%     
%     % Sender meldingen om basens hastighet og retning
%     send(pub_vel,msg_vel)
%    
%     % Henter basens rotasjon og posisjon:
%     % Posisjon
%     odomMsg = receive(sub_odom,3);
%     pose = odomMsg.Pose.Pose;
%     x = -pose.Position.X;
%     y = -pose.Position.Y;
%     % Rotasjon
%     quat = pose.Orientation;
%     angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%    
%     robotCurrentPose = [x;y;angles(1)] % Oppdaterer pose
%     distanceToGoal = norm(robotCurrentPose(1:2) - robotMaalpos(:)) % Oppdaterer avstand tl målet 
%     
%     % Oppdaterer plottet 
%     hold off
%     
%     % Simulerer bevegelsen av roboten
%     lp2.plot()
%     hold all
%     plot(path_plot(:,1), path_plot(:,2),'k--d')
%    
%     % Plotter banen til roboten
%     plotTrVec = [robotCurrentPose(1:2); 0]*10;
%     plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
%     plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize) % Må ha groundvehicle.stl ikke robot_base.stl
%     light;
%     xlim([0 170])
%     ylim([0 120])
% 
%         waitfor(vizRate);
% end
% 
% % Stopper roboten
% msg_vel.Linear.X = 0.0;
% msg_vel.Angular.Z = 0.0;
% 
% send(pub_vel,msg_vel) % Sender melding 
% 
% 
% %% Vanner plante 
% % Hvor mange ganger i sekundet komandoene skal bli sendt
% rate = robotics.Rate(10);
% kjor = true;
% 
% while kjor
% odom.Pose.Pose.Position
%     % Går gjennom banen for armen 
%     
%     %Fra start til plante
%     if index1 <= q1_moves
%         q_temp = m1(index1,:);
%         index1 = index1 +1;  
%         
%     %Fra plante til kjøre
%     elseif index2 <= q2_moves
%         if index2 == 1 
%             pause(2)
%         end
%         q_temp = m2(index2,:);
%         index2 =  index2 +1;  
%     
%     else
%         index1 = 1;
%         index2 = 1;
%         kjor = false;
%     end
%     
%     % Vinkelverdier for hvert ledd 
%     msg_q1.Data = q_temp(1);
%     msg_q2.Data = q_temp(2);
%     msg_q3.Data = q_temp(3);
%     msg_q4.Data = q_temp(4);
%     msg_q5.Data = q_temp(5);   
%     
%     % Sender meldingene om vinklene ut til ros
%     send(pub_q1,msg_q1)
%     send(pub_q2,msg_q2)
%     send(pub_q3,msg_q3)
%     send(pub_q4,msg_q4) 
%     send(pub_q5,msg_q5)
%      
%     waitfor(rate);
% end
% 
% rosshutdown % Stopper ros
% 
% function odom_callback(src,msg)
%     global odom
%     odom = msg; 
% end