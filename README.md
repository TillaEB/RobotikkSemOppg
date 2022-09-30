# Semesteroppgave for ELE306 Robotikk -LOGG

## Fredag 30.09 
I dag har vi jobbet med PDR, og er snart ferdig med den. 
Under diskusjonen i forbindelse med PDR tok vi igjen og vurderte hvilken navigasjonsmetode vi syntes passet best. 
![image](https://user-images.githubusercontent.com/112081507/193261368-ae845402-f022-45de-856e-cf3b8eafd746.png)
Vi konkluderer fermdeles med at Lattice er et godt valg for vårt formål 

## Torsdag 29.09
ENDELIG 
Mangler litt men snart ferdig kart 
![image](https://user-images.githubusercontent.com/112081507/192988671-f8e8f267-55dc-44c2-944f-ce75fed773c7.png)

KODE FOR Å LAGE KJØPESENTER MATRISEN 
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

contour(kjopesenter) 


%JIPPI! FERDIG 


## Tirsdag 27.09 
Funnet ut hvordan vi kan endre utseende på huset slik at vi kan tilpasse det til et kjøpesenter. Nå må vi finne ut størrelsesforholdene 
![image](https://user-images.githubusercontent.com/112081507/192486573-a000cb86-703a-441c-aeb4-38696940a00d.png)
![image](https://user-images.githubusercontent.com/112081691/192516434-99699bd3-932c-4c35-99fb-92e269a5ec99.png)

Link til hvordan lattice koden er bygget opp 
https://github.com/petercorke/robotics-toolbox-matlab/blob/master/Lattice.m 


## Søndag 25.09
Blitt ferdig med oblig og gjort klart hvilke navigasjon- og kontrollmetode vi skal bruke. Samtidig snakket smått om flere ting vi er litt usikre på som vi som planlagt skal ta opp med foreleser tirsdag.

## Fredag 23.09
Jobbet videre med oblig og fått til oppgave 2 og oppgave 4. 
Har fått svar fra foreleser ang møte om en del spørsmål. Dette møtet blir tirsdag etter kl 12.00.
Vi har bestemt oss for navigasjonsmetoden lattice planner. 

## Torsdag 22.09
Sendte i går melding til foreleser da vi var usikker på hvordan vi skulle nå flere målpunkt. Om vi skulle behandle det som mange små strekkninger med mål og startpunkt eller et målpunkt med flere startpunkt. Etter veldig godt svar og tips fra foreleser skjønte vi mer hvordan vi måtte tenke. 
Vi må gå for roadmap method og ønsker å ta hensyn til kinematikken da vi velger en såpass avansert robot. Da gjennstår det bare to navigasjonsmetoder som vi må lese på og bestemme oss for, lattice og RRT 

Plan for dagen:
 - Tegne skjematisk tegning av basen til roboten (oppg 2)
 - Utvikle likninger for omni (oppg 3)
 - Holonom/ikke holonom begrunnelse (oppg 4) 
 
 Link til Robotino deler hardware:
 https://www.festo-didactic.com/int-en/services/robotino/hardware/?fbid=aW50LmVuLjU1Ny4xNy4zNC4xNDI5
 
 Link til youtube video 
 https://www.youtube.com/c/TommyHvidsten/videos
 https://www.youtube.com/watch?v=wwQQnSWqB7A
 
## Onsdag 21.09 
Forenklet kart av senteret med hindringer og dører. 
<img width="689" alt="image" src="https://user-images.githubusercontent.com/112080695/191510127-eede91f6-79f2-4d07-9921-542b72f281b3.png">

Vi diskuterer å bruke omnidirectional(omni) robot, og da bruke og da bruke "readily available Robotino models for ROS simulations". 
<img width="521" alt="image" src="https://user-images.githubusercontent.com/112080695/191513361-a308e9b5-b575-4735-aa56-7290b7a337e9.png">
![image](https://user-images.githubusercontent.com/112080695/191513508-56013520-5b01-4cbd-8802-cd4f8ab7b709.png)
https://robots.ros.org/robotino/


Søppel går som uplanlagt hindring

### Omnidirectional robot 
https://www.researchgate.net/publication/221786657_Omnidirectional_Mobile_Robot_-_Design_and_Implementation 

## Tirsdag 20.09 
Diskusjon angående hvilken type robot vi bør velge. 
Artikkelen under peker på problematikk ved "differential steering mobile robot(diff)"
http://www.robotplatform.com/knowledge/Classification_of_Robots/wheel_control_theory.html

Vi bestemte oss for å bruke 1.etg i Bergen Storsenter som mal til kartet. Gikk på senteret for å kartlegge hindringer. 


## Søndag 18.09 
Undersøkt en del for å finne ut hvilken type robot vi burde velge, "car-like" eller "differential steering mobile robot(diff)". Med en differentialt styrt robot vil vi kunne ha muligheten til å snu roboten uten at den beveger seg fremover, noe som kan være hensiktsmessig dersom den skulle manuvrere gjennom f.eks et område med stoler osv. Pros ved å bruke differential steering og unicycle model er som sagt muligheten til å kunne snu seg, samt at den skal være veldig vanlig å benytte og enkelt å estimere og kontrollere posisjonen. Et annet nettsted peker på at det kan være vanskelig å styre rett frem og at den kanskje ikke kjører og snur helt som forventet. Vår robot er tenkt å kjøre etter kjøpesenteret er stengt og så vurderer i første omgang at dette ikke vil være et stort problem så lenge den når målet. Et annet nettsted peker også på at nybegynnere burde brukke diff som hentyder at det er en enkel/enklere type robot. 


## Torsdag 15.09
Veiledningsmøte med forelesere
Vi lurer på følgende ting: Vil det være flere veiledningssituasjoner og hvor ofte vil disse være. I forhold til MATLAB, skal man skrive egen kode eller bearbeide/bruke skript lagt ut på Canvas? 

Fra møtet med veilederer: 
Navigasjonsstrategier kan vises hver for seg i matlab. Vi kan modifisere skriptene som er lagt ut, og kan endre på huset i toolboxen slik at det fremstiller et kjøpesenter. 
Per nå er planen å ha en robot per etasje dersom kjøpesenteret går over flere plan. Arbeidet med oblig1 vil gjennspeile mye av det som skal være med i PDR

Fra forelesning: 
PDR trenger ikke være endelig løsning. Skal hansle om tema, hvordan vi tolker oppgaven, idder vi har hatt, samt ta opp noen ting fra kravdokumentet. 


## Tirsdag 06.09 
I dag valgte vi oppgave. Vi har valgt å lage en robot som skal vanne planter enten i hjemmet eller på f.eks et kjøpesenter.
Vi har tenkt mest på en robot som er tenkt for et kjøpesenter. 


## Torsdag 25.08
I dag har vi opprettet GitHub for semesteroppgaven. Vi tenker å skrive oppgaven online versjonen av Word. 
Vi har vært innom tanken på oppgave om robot som vanner planter på kjøpesenter og gaffeltruck. Ikke bestemt enda.

