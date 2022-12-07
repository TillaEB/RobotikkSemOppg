%% Simulering av arm i matlab 

clc; clear all;
%% Lager arm:
import ETS3.*

% Lengder på ledd 
B= 200 + 600;    % 80 cm 
L1 = 500;        % 40 cm 
L2 = 500;        % 40 cm 
L3 = 500;        % 40 cm 
L4 = 500;        % 40 cm  
L5 = 350;        % 25 cm 

% DH parametere
j1 = Revolute('d',L1,'a',0,'alpha', -pi/2, 'offset',0)
j2 = Revolute('d',0,'a',L2,'alpha', 0, 'offset',-pi/2)
j3 = Revolute('d',0,'a',L3,'alpha', 0, 'offset',0)
j4 = Revolute('d',0,'a',L4,'alpha', 0, 'offset',0)
j5 = Revolute('d',0,'a',L5,'alpha', 0, 'offset',0)

% Setter sammen DH parametere for å lage roboten 
H2rObot_arm = SerialLink([j1 j2 j3 j4 j5],'name','H2rObot')

%% Arm posisjoner: 
% Armen er 180 over bakken når den er i denne posisjonen  
q_se_etter_plante_venstre = [pi/4,0,deg2rad(80),0,0]; % Armvinkler for å se etter plante venstre
q_se_etter_plante_hoyre = [-pi/4,0,deg2rad(80),0,0]; % Armvinkler for å se etter plante høyre
q_kjore = [0,0,pi/2,pi/2,0]; % Armvinkler for kjøre posisjon 

% Transformasjonsmatrise for start posisjonen fra forwad kinematikk 
T_H2rObot_O = H2rObot_arm.fkine([0,0,0,0,0]) 

% Transformasjonsmatrise for base til end-effector i 
% se etter plante venstre posisjon  
BTPV = H2rObot_arm.fkine(q_se_etter_plante_venstre)

% Transformasjonsmatrise for base til end-effector i 
% se etter plante høyre posisjon 
BTPH = H2rObot_arm.fkine(q_se_etter_plante_hoyre)

% Kameraet sitter 60 mm (6 cm) langs x-aksen, og 110 mm (11 cm) ut fra
% midten av end-effectoren 
ETK = transl(-60,110,0) 

% Kameraet ser en plante 150 mm (15 cm) nedover (y-retning) og 70 mm (7 cm)
% til høyre (z-retning) 
KTPhoy = transl(0,150,-70)  %% Planten er 150 cm over bakken

% Kameraet ser en plante 1495 mm (149,5 cm) (maks) nedover (y-retning) 
% og 150 mm (15 cm) innover mot basen (x-retning) 
KTPlav = transl(-150,1450,0) % Planten er 30,5 cm over bakken

% Transformasjonsmatrise for base til vanne plante høy venstre 
BTVP_Hoyv = BTPV.T * ETK * KTPhoy 

% Transformasjonsmatrise for base til vanne plante høy høyre 
BTVP_Hoyh = BTPH.T * ETK * KTPhoy 

% Transformasjonsmatrise for base til vanne plante lav venstre 
BTVP_lavv = BTPV.T * ETK * KTPlav

% Transformasjonsmatrise for base til vanne plante lav høyre 
BTVP_lavh = BTPH.T * ETK * KTPlav


% Fordi vi har en underaktuert robotarm har vi valg å ignorere rx, ry og rz

% Vinkler for start posisjon
q0 = H2rObot_arm.ikine(T_H2rObot_O,'mask',[1 1 1 0 0 0])

% Vinkler for vanning av plante høy venstre 
q1 = H2rObot_arm.ikine(BTVP_Hoyv,'q0', q0,'mask',[1 1 1 0 0 0])

% Vinkler for vanning av plante lav venstre 
q2 = H2rObot_arm.ikine(BTVP_lavv,'q0', q0,'mask',[1 1 1 0 0 0])

% Vinkler for vanning av plante høy høyre 
q3 = H2rObot_arm.ikine(BTVP_Hoyh,'q0', q0,'mask',[1 1 1 0 0 0])

% Vinkler for vanning av plante lav høyre 
q4 = H2rObot_arm.ikine(BTVP_lavh,'q0', q0,'mask',[1 1 1 0 0 0])


% Lager bane:
t = [0:0.05:2]'; % tidssteg
  
m0 = mtraj(@tpoly, q0,q_kjore,t);                                % Bane fra startposisjon til kjøreposisjon
m1 = mtraj(@tpoly, q_kjore, q_se_etter_plante_venstre, t);       % Bane fra kjøreposisjon til se-etter-plante posisjon venstre
m2 = mtraj(@tpoly, q_se_etter_plante_venstre, q1, t);            % Bane fra se-etter-plante posisjon venstre til vanne plante høy venstre posisjon
m3 = mtraj(@tpoly, q1, q_kjore, t);                              % Bane fra vanne plante høy venstre posisjon til kjøreposisjon 
m4 = mtraj(@tpoly, q_kjore, q_se_etter_plante_venstre, t);       % Bane fra kjøreposisjon til se-etter-plante posisjon venstre
m5 = mtraj(@tpoly, q_se_etter_plante_venstre,q2, t);             % Bane fra se-etter-plante posisjon venstre til vanne plante lav venstre posisjon
m6 = mtraj(@tpoly, q2, q_kjore, t);                              % Banse fra vanne plante lav venstre posisjon til kjøreposisjon 

m7 = mtraj(@tpoly, q_kjore, q_se_etter_plante_hoyre, t);         % Bane fra kjøreposisjon til se-etter-plante posisjon høyre
m8 = mtraj(@tpoly, q_se_etter_plante_hoyre, q3, t);              % Bane fra se-etter-plante posisjon høyre til vanne plante høy høyre posisjon
m9 = mtraj(@tpoly, q3, q_kjore, t);                              % Bane fra vanne plante høy høyre posisjon til kjøreposisjon 
m10 = mtraj(@tpoly, q_kjore, q_se_etter_plante_hoyre, t);        % Bane fra kjøreposisjon til se-etter-plante posisjon høyre
m11 = mtraj(@tpoly, q_se_etter_plante_hoyre,q4, t);              % Bane fra se-etter-plante posisjon høyre til vanne plante lav høyre posisjon
m12 = mtraj(@tpoly, q4, q_kjore, t);                             % Banse fra vanne plante lav høyre posisjon til kjøreposisjon 

% % %Simulering av arm bevegelse: 
H2rObot_arm.plot(m0) 
H2rObot_arm.plot(m1)  
H2rObot_arm.plot(m2) 
H2rObot_arm.plot(m3) 
H2rObot_arm.plot(m4) 
H2rObot_arm.plot(m5) 
H2rObot_arm.plot(m6) 
H2rObot_arm.plot(m7) 
H2rObot_arm.plot(m8)  
H2rObot_arm.plot(m9) 
H2rObot_arm.plot(m10) 
H2rObot_arm.plot(m11) 
H2rObot_arm.plot(m12)  
