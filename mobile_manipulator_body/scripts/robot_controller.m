%% ELE306 Robotikk  semesteroppgave  
%% Gruppe 2

clear,close all;
% Avslutter og starter opp ROS
rosshutdown
rosinit

% Arm controller
pub_joints = rospublisher ('/arm_controller/command','trajectory_msgs/JointTrajectory');

%% Venstre side 
% Point 1 kjøreposisjon
tjPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint1.Positions = [pi/2,0,-pi/2,pi,0];               % Setter posisjon
tjPoint1.Velocities = zeros(1,5);
tjPoint1.TimeFromStart.Sec = 5;

% Point 2 Se etter plante venstre 
tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint2.Positions = [(6*pi)/8,0,-pi/2,pi,0];           % Setter posisjoner
tjPoint2.Velocities = zeros(1,5);
tjPoint2.TimeFromStart.Sec = 10;

% Point 3 Høy plante venstre  
tjPoint3 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint3.Positions = [(6*pi)/8,-0.1745,-pi/2,pi,0];     % Setter posisjoner
tjPoint3.Velocities = zeros(1,5);
tjPoint3.TimeFromStart.Sec = 15;

% Point 4 kjøreposisjon
tjPoint4 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint4.Positions = [pi/2,0,-pi/2,pi,0];               % Setter posisjoner
tjPoint4.Velocities = zeros(1,5);
tjPoint4.TimeFromStart.Sec = 20;

% Point 5 Se etter plante venstre 
tjPoint5 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint5.Positions = [(6*pi)/8,0,-pi/2,pi,0];           % Setter posisjoner
tjPoint5.Velocities = zeros(1,5);
tjPoint5.TimeFromStart.Sec = 25;

 % Point 6 lav plante venstre
 tjPoint6 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
 tjPoint6.Positions = [(6*pi)/8,-0.8727,-pi/2,pi,0];    % Setter posisjoner
 tjPoint6.Velocities = zeros(1,5);
 tjPoint6.TimeFromStart.Sec = 30;
 
 % Point 7 kjøreposisjon
tjPoint7 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint7.Positions = [pi/2,0,-pi/2,pi,0];               % Setter posisjoner
tjPoint7.Velocities = zeros(1,5);
tjPoint7.TimeFromStart.Sec = 35;

%% Høyre 
% Point 8 Se etter plante høyre 
tjPoint8 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint8.Positions = [pi/4,0,-pi/2,pi,0];               % Setter posisjoner
tjPoint8.Velocities = zeros(1,5);
tjPoint8.TimeFromStart.Sec = 40;

 % Point 9 Høy plante høyre  
tjPoint9 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint9.Positions = [pi/4,-0.1745,-pi/2,pi,0];         % Setter posisjoner
tjPoint9.Velocities = zeros(1,5);
tjPoint9.TimeFromStart.Sec = 45;

 % Point 7 kjøreposisjon
tjPoint10 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint10.Positions = [pi/2,0,-pi/2,pi,0];              % Setter posisjoner
tjPoint10.Velocities = zeros(1,5);
tjPoint10.TimeFromStart.Sec = 50;

% Point 8 Se etter plante høyre 
tjPoint11 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint11.Positions = [pi/4,0,-pi/2,pi,0];              % Setter posisjoner
tjPoint11.Velocities = zeros(1,5);
tjPoint11.TimeFromStart.Sec = 55;

% Point 12 lav plante høyre
 tjPoint12 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
 tjPoint12.Positions = [pi/4,-0.8727,-pi/2,pi,0];       % Setter posisjoner
 tjPoint12.Velocities = zeros(1,5);
 tjPoint12.TimeFromStart.Sec = 60;
 
% Point 13 kjøreposisjon
tjPoint13 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint13.Positions = [pi/2,0,-pi/2,pi,0];              % Setter posisjoner
tjPoint13.Velocities = zeros(1,5);
tjPoint13.TimeFromStart.Sec = 65;
 

msg = rosmessage(pub_joints);

% Referer til ledd-navn til roboten
msg.JointNames = {'arm_base_joint' 'shoulder_joint' 'bottom_wrist_joint' 'elbow_joint' 'top_wrist_joint' };

% Punktene den skal følge
msg.Points = [tjPoint1,tjPoint2,tjPoint3,tjPoint4,tjPoint5,tjPoint6,tjPoint7,tjPoint8,tjPoint9,tjPoint10,tjPoint11,tjPoint12,tjPoint13]

% Sender melding om armens retning og hastighet
send(pub_joints,msg)