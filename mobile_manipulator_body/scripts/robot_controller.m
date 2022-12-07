clear,close all;
rosshutdown
rosinit

pub_joints = rospublisher ('/arm_controller/command','trajectory_msgs/JointTrajectory');

% Point 2 kjøreposisjon
tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint2.Positions = [-pi/2,0,pi/2,0,0];
tjPoint2.Velocities = zeros(1,5);
tjPoint2.TimeFromStart.Sec = 5;

% Point 3 se etter plante 
 tjPoint3 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
 tjPoint3.Positions = [0,0,deg2rad(75),0,0];
 tjPoint3.Velocities = zeros(1,5);
 tjPoint3.TimeFromStart.Sec = 8;
 
% Point 4 høy plante venstre
tjPoint4 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint4.Positions = [0, pi/12, deg2rad(80), 0, 0] %[-0.0706,-0.0317,1.3658,0.5129,0.1112];
tjPoint4.Velocities = zeros(1,5);
tjPoint4.TimeFromStart.Sec = 12;

% % Point 5 kjøreposisjon
%  tjPoint5 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%  tjPoint5.Positions = [-pi/2,0,pi/2,0,0];
%  tjPoint5.Velocities = zeros(1,5);
%  tjPoint5.TimeFromStart.Sec = 16;
%  
%  % Point 6 se etter plante
%  tjPoint6 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%  tjPoint6.Positions = [0,0,deg2rad(75),0,0];
%  tjPoint6.Velocities = zeros(1,5);
%  tjPoint6.TimeFromStart.Sec = 20;
%  
%  % Point 7 lav plante venstre
%  tjPoint7 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%  tjPoint7.Positions = [0,1.0472,1.0472,0,0.-pi/12]%[-0.0000,2.2245,0.0551,-0.1270,0.1583];
%  tjPoint7.Velocities = zeros(1,5);
%  tjPoint7.TimeFromStart.Sec = 24;
%  
%  % Point 8 kjøreposisjon
%  tjPoint8 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%  tjPoint8.Positions = [-pi/2,0,pi/2,0,0];
%  tjPoint8.Velocities = zeros(1,5);
%  tjPoint8.TimeFromStart.Sec = 28;
%  

msg = rosmessage(pub_joints);
msg.JointNames = {'arm_base_joint' 'shoulder_joint' 'bottom_wrist_joint' 'elbow_joint' 'top_wrist_joint' };

msg.Points = [tjPoint2,tjPoint3,tjPoint4] %,tjPoint5,tjPoint6,tjPoint7,tjPoint8]

send(pub_joints,msg)


