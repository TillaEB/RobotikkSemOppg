%% Simulering av arm i matlab 

clc; clear all;
%% Lager arm:
import ETS3.*

%% Forward kinematics 
% Lengder på ledd 
% B= 600;          % 60 cm 
L1 = 0.5;        % 50 cm 
L2 = 0.5;        % 50 cm 
L3 = 0.5;        % 50 cm 
L4 = 0.5;        % 50 cm  
L5 = 0.35;        % 35 cm 

% DH parametere
j1 = Revolute('d',L1,'a',0,'alpha', -pi/2, 'offset',0);
j2 = Revolute('d',0,'a',L2,'alpha', 0, 'offset',-pi/2);
j3 = Revolute('d',0,'a',L3,'alpha', 0, 'offset',0);
j4 = Revolute('d',0,'a',L4,'alpha', 0, 'offset',0);
j5 = Revolute('d',0,'a',L5,'alpha', 0, 'offset',0)

% Setter sammen DH parametere for å lage roboten, for de fire første
% leddene 
H2rObot_arm = SerialLink([j1 j2 j3 j4],'name','H2rObot')
H2rObot_arm.plot([0 0 0 0])

%Transformasjonsmatriser regnet ut for hånd 
T01=[1 0 0 0;0 0 1 0;0 -1 0 0.5; 0 0 0 1]
T12=[0 1 0 0; -1 0 0 -0.5; 0 0 1 0; 0 0 0 1]
T23=[1 0 0 0.5;0 1 0 0;0 0 1 0;0 0 0 1]
T34=[1 0 0 0.5;0 1 0 0;0 0 1 0; 0 0 0 1]
T04_regne=T01*T12*T23*T34

%Sjekker svaret mot fkine 
T04=H2rObot_arm.fkine([0 0 0 0])

%% Differential kinematics 
%Konfigurasjoner vi skal teste (nå for alle fem leddene)

H2rObot_arm = SerialLink([j1 j2 j3 j4 j5],'name','H2rObot')


q_kjore = [-pi/2,0,pi/2,pi/2,0];            % Armvinkler for kjøre posisjon 
q_se_etter_plante_venstre = [0,0,pi/2,0,0]; % Armvinkler for å se etter plante venstre

J0=H2rObot_arm.jacob0(q_kjore)
rank=rank(J0)
jsingu(J0)
H2rObot_arm.teach(q_kjore,'callback',@(r,q) r.fellipse(q),'view','x')
H2rObot_arm.teach(q_se_etter_plante_venstre,'callback',@(r,q) r.fellipse(q),'view','x')

