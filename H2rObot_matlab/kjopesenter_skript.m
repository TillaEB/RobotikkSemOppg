
%%Lager kjopesenter 

%Lager tom matrise for å illustrere kjøpesenteret 
kjopesenter=zeros(80,170);

%Ligger inn okkuperte områder 
%Starter fra venstre og okuperer

%Ytterkant oppe  
kjopesenter(79:80,[1:100])=1;

%Vinmonopolet(101) og deler av Meny(102) og litt heis 103 
kjopesenter(30:80,[1:50])=1;     %(fra_rad:til_rad,[fra_kolonne:til_kolonne])
kjopesenter(25:35,[43:55])=1; 
%Heis
kjopesenter(23:25,[43:55])=1;

%Resten av Meny og litt esspressohouse (104)
kjopesenter(1:15,[1:55])=1;
kjopesenter(15:30,[1:27])=1;
kjopesenter(1:5,[27:55])=1;

%Ytterkant nede 
kjopesenter(1:2,[55:160])=1;

%Ytterkant venstre 
kjopesenter(5:80,[1:2])=1;


%Coop Extra (106)
kjopesenter(38:80,[90:110])=1;


%3-> Sabrura 
kjopesenter(45:70,[121:130])=1;

%Rulletrapp Sabrura
kjopesenter(39:45,[123:128])=1; 

%4->Blomsterbutikk 
kjopesenter(45:70,[137:150])=1;

%Rulletrapp Blomsterbutikk 
kjopesenter(39:45,[140:147])=1;

% Burgerking, Telenor, Cutters 
kjopesenter(3:29,[94:119])=1;

%Bigbite 
kjopesenter(21:30,[125:135])=1;

%Smoothie under rulletrapp 
kjopesenter(50:59,[65:76])=1;

%Rulletrapp over smoothie
kjopesenter(30:51,[65:72])=1;

%Narvesen 
kjopesenter(1:15,[136:170])=1;

%Kjøttbutikk og bildebutikk og apotek 
kjopesenter(4:50,[157:170])=1;

% %Ramme høyrevegg 
% kjopesenter(1:56,[164:170])=1;

 %Ramme topp blomsterbutikk 
 kjopesenter(70:80,[110:170])=1;

 kjopesenter(60:70,[150:170])=1;

 %Hindringer
kjopesenter(23:24,[58:59])=1;
kjopesenter(33:34,[57:58])=1;
kjopesenter(43:44,[57:58])=1;
kjopesenter(50:52,[57:58])=1;
kjopesenter(70:71,[57:58])=1;
kjopesenter(70:72,[57:58])=1;
kjopesenter(70:71,[54:55])=1;

kjopesenter(42:43,[85:87])=1;
kjopesenter(54:55,[85:87])=1;
kjopesenter(60:62,[85:87])=1;
kjopesenter(66:67,[85:87])=1;

kjopesenter(10:11,[65:66])=1;
kjopesenter(8:9,[84:85])=1;
kjopesenter(29:30,[90:91])=1;
kjopesenter(41:42,[118:119])=1;

kjopesenter(8:9,[122:123])=1;
kjopesenter(8:9,[132:133])=1;
kjopesenter(18:19,[140:141])=1;
kjopesenter(26:27,[153:154])=1;

%Dører
kjopesenter(1:3,[73:77])=1;
kjopesenter(78:80,[73:77])=1;
kjopesenter(1:3,[126:130])=1;

%Kan se konturen av occupansygrid
% contour(kjopesenter); 

% Punkter den skal innom 
%Gamle 
P1=[35,21,0];
P2=[56,66,0];
P3=[80,57,0];
P4=[104,33,0];
P5=[119,39,0];
P6=[128,18,0];
P7=[140,24,0];
MP=[152,39,1.571];
P8=[152,54,1.571];
%Endrete 
% P1=[35,21,0];
% P2=[62,57,0];
% P3=[80,57,0];
% P4=[104,33,0];
% P5=[119,39,0];
% P6=[128,18,0];
% P7=[140,24,0];
% MP=[152,39,1.571];
% P8=[152,54,1.571];
