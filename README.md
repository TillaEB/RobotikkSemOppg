# Semesteroppgave for ELE306 Robotikk -LOGG



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

