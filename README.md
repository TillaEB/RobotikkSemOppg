# Semesteroppgave for ELE306 Robotikk 🤖 
## Tirsdag 25.10 
Today is arm day 
Vi jobber med arm til roboten. Kravet er at den skal nå potter som står på gulvet helt ned til 20cm og potter opp til 150cm. Basen vår er 40cm høy, inklusivt 10 cm radius på hjulene vil dette gi at basen på armen er festet 50 cm over gulvet. 
Ut i fra hvordan vi har tenkt at roboten skal kjøre opp til planetene vil det variere om planten havner på høyre eller venstre siden på basen. Altså må vi har et ledd med rotasjon rundt z aksen (koordinatsystem 0 vil være likt som body corrdinate til roboten) 

Armen skal kun bevege seg fra venstre til høyre og motsatt via fronten på roboten. Dette fordi at det ikke skal bli konflikt mellom vanntanken og armen. Armen kan kun bevege seg mellom 0 og pi grader 
![image](https://user-images.githubusercontent.com/112081507/197809867-5bff5241-7db5-4e70-a6a1-6b58dde80460.png)

Brukt veldig vitenskapelige metoder for finne ut om robotarmen 
![image](https://user-images.githubusercontent.com/112081507/197810755-1f66b638-53d1-41b8-ab6a-40ea59bb4cf7.png)

Spørsmål til hjelpetime 

- Robotarmen, skal man designe selv eller finne en som allerede finnes 
- Hvordan er det å tegne en robotarm i ros, vanskelig? 
- I følge lattice skal roboten egentlig klare å kjøre ruten, men når vi simulerer kommer den noen ganger ikke frem til målet. Vi har noen ruter å vise til hvor den klarer det fint. Hvis det er viktig at den klarer å kjøre mellom alle plantene trenger vi hjelp. 


## Mandag 17.10
Begynte å skrive inn det vi hadde fra Oblig 1 på rapporten, og så på hvilke sensorer vi trenger. 

-Kamera sensor for å identifisere eventuelle hindringer i veibanen, og finne frem til riktig sted den skal vanne.
-Fuktighetssensor til å måle fuktigheten i jorden til planten.
-Vektsensor for å kunne detektere om vanntanken er tom/full.

Gikk for Intel RealSense D415. Dette kamerasystemet egner seg godt til vår bruk, og har innebygget RGB fargesensor. Selv om vi ikke skal bruke RGB i denne oppgaven, vil det gi mulighet til å videreutvikle roboten, om ønsket, til å kunne plukke vekk vissne blader uten å måtte skifte kamera. 



## Torsdag 13.10 

Waterino--> H2rObot

Fått kartet inn i fusion som 3D model: 

<img width="549" alt="image" src="https://user-images.githubusercontent.com/112080695/195574469-ef51f6e5-53c1-46ca-8b14-ed7fdeec14b8.png">

Se stl fil. 
For å få inn i fusion: mesh - insert mesh (med blå pil over create) - velg stl fil (ferdig skalert)


## Onsdag 12.10 
Arbeider i dag med å få kjøpesenteret inn i fusion 360. Dette har vi nå klart, nå handler det om skalering. 
Ser også på å få stl fil fra fusion360 inn i ROS så vi kan lage en ny verden. 
Link vi kanskje kan bruke: https://classic.gazebosim.org/tutorials?tut=build_model&cat=build_robot  

Ble tipset om IMU i forelesning i dag. Brukes til dead reckogning, som vi skal ha

## Tirsdag 11.10
Vi har blitt enige om at punktene vi plotter i MatLab er området rundt planten, og ikke nødvendigvis akuratt der planten står. Roboten kjører derfor til området rundt planten, mens robotarmen vil ta ansvar for strekningen til den faktiske planten. Siden vi har byttet til car-like robot vil vi at roboten skal komme "på siden" av planten, og ikke peke nesen mot planten. Dette er mest praktisk i forhold til at den skal kjøre videre etter vanning.
Fortvil ikke, det går fint å svinge 😏 :
![image](https://user-images.githubusercontent.com/112081691/195098064-5a73eb01-1522-45e3-84ce-fd3770682631.png)

Vi har nå funnet ut og forstått mer av hvordan vi skal navigere roboten i lattice planner i Matlab - og knukket koden for hvordan θ fungerer. Nå gir alt mening.

<img width="508" alt="image" src="https://user-images.githubusercontent.com/112080695/198144718-d327b7b3-8691-4741-ad1d-e036dd500247.png">

## Torsdag 06.10
Samme uke, ny diskusjon rundt navigasjonsmetode.
Etter mye om og men har vi kommet fram til at det er mer fornuftig å velge car-like robot. Vi fortsetter med samme lokaliseringsmetode, men endrer controller til "follow a trajectory".
Vi har funnet ut at det ikke er nødvendig å bruke omnidirectional robot ettersom at vi har et stort kjøpesenter som roboten skal navigere i. Vi føler også at omnidirectioal robot blir for komplekst og fjernt, og bytter derfor til en robot som vi kan ha mer eierskap over, som vi forsår oss mer på. 
Arbeidet vi har gjort frem til nå vil likevel ikke være bortkastet. Vi har brukt mye tid på latticeplanner tidligere, og har nå behov for en navigasjonsmetode som krever at man tar hensyn til robotens kinematikk, derfor vil vi fortsette arbeidet med lattice planner. 

Jobbet med lattice for å skjønne hvordan de forskjellige funksjonene funker, root, grid, inflate gir endelig mening. 
Forsket frem og tilbake for å finne ut hvilke parametre som ga oss et best mulig resultat. 
For å unngå at lattice grid går helt inntil hindringer brukte vi inflate=1. Dette gir en "dødsone" på 1 meter fra hindringene. Noen steder hvor det er stolper i kartet går strekene tvers over hindringer. Dette har vi ikke klart å unngå. Noen steder vil den også kollidere med hjørne av en butikk osv.  

Måten vi har tatt hensyn til dette: 
 - Vi har sørget for å legge inn mellompunkt noen steder hvor det kreves slik at roboten ikke kjører en hindring. 
 - Det koster mer å svinge enn å kjøre rett, prøver å holde den mest mulig "midt" i kartet og på en linje. 
 - Tilpasset hvilken vinkel den skal ende opp i. 
 
![image](https://user-images.githubusercontent.com/112081507/195333800-16748f94-4809-4dac-a0dd-3eadff4591dc.png)


Vi velger Grid=3 for dette gir et fornuftig antall noder. Griddet ligger da opp til at roboten snur 180 grader via en halvsirkel med radius 1.5meter. Vår robot har en avstand på 0.60m (cc) mellom hjulene (wheelbase) og en maksimal sving på forhulene på 40 grader. 
Turning radius for vår robot blir da 
Tr=wb/tan(maksSving)=0.60m/tan(40 deg)=0.7149m. 
Dette betyr at roboten vår ikke vil ha problemer med å utføre en 180 graders sving slik som grid er satt opp.

Rootnode satt vi i midten for at den skulle bre seg utover. Å sitte den i et f.eks et hjørne ga mye færre noder, og veldig rart plasserte noder. 
Vi har tenkt oss 8 "soner" hvor det vil befinne seg planter, og har testet at roboten kjører en kollisjonsfri rute mellom plantene. 
Roboten vil kjøre en fast rute, og ha en fast rekkefølge den besøker plantene i. 

## Onsdag 05.10 
Tilla har fikset skalering på bildet slik at kjøpesenteret ikke er 1.6 km bredt lengre.
Har gått gjennom punktene til oppgaven for å skaffe oss en bedre oversikt over hva som skal være med og fremdrift i oppgaven. Ligger tilsynelatende greit ann i forhold til fremdrift i faget. Fokus fremover vil være matlab og simuleringer. Vi må lese oss opp på sensorer og kontrollstrategi, og få bestemt oss for det i løpet av kort tid. 


## Tirsdag 04.10 
Ny uke, ny diskusjon rundt navigasjonsmetode. 
Vi har lenge vært låst på lattice planner, men ettersom at roboten vår ikke har naturlig svingradius er dette noe vi må legge inn selv. RRT blir overkill for en holonomrobot. Vi ønsker å bruke roadmap methods, da står vi igjen med en siste metode, nemlig PRM, som er der vi lander for denne gang. (Dette avviker fra det vi har levert inn på obligen) 
Senere i uken er det PDR vi må derfor gjøre litt om på det vi har gjort til nå. 

Vi innser at vi har letet etter gode grunner til å bruke lattice og ikke klart å være helt objektiv i diskusjonen. 
Neste steg nå er å se hvor mange noder vi trenger for å dekke en tilfredsstillende del av kartet. 

### MATLAB 
- Hvordan gå fra bilde til occupancy grid: 
https://se.mathworks.com/help/nav/ref/binaryoccupancymap.html

Se fil: bildekart.m for vårt kart. 
Orginalt kart: se fil sistekart.

Occupancy grid:

<img width="622" alt="image" src="https://user-images.githubusercontent.com/112080695/194028460-3c6d7fb2-220f-4006-a078-172df5f3e214.png">

- PRM i matlab: 
https://se.mathworks.com/help/robotics/ug/path-planning-in-environments-of-difference-complexity.html

Se også https://se.mathworks.com/help/robotics/ref/mobilerobotprm.html


Se fil PRMpathfinder.m for vår simulering

PRM planlegging:

<img width="636" alt="image" src="https://user-images.githubusercontent.com/112080695/194025442-64d630f1-55f3-49b4-a40b-9568f11d74af.png">

Kart med punkter for planter:

![image](https://user-images.githubusercontent.com/112080695/194028014-69e30c51-5896-4134-a2e4-04ad769adc12.png)

- Planleggings metoder i matlab: 
https://se.mathworks.com/help/nav/motion-planning.html

Se under motion planning. 

## Fredag 30.09 
I dag har vi jobbet med PDR, og er snart ferdig med den. 
Under diskusjonen tok vi oppigjen tråden angående navigasjonsmetode
![image](https://user-images.githubusercontent.com/112081691/193263608-ea6541b9-7457-4e71-a353-1063b812962c.png)
Argumenter hvor de tre metodene er like gode har vi ikke tatt med.
Vi konkluderer fermdeles med at Lattice er et godt valg for vårt formål 

## Torsdag 29.09
ENDELIG 
Mangler litt men snart ferdig kart 
![image](https://user-images.githubusercontent.com/112081507/192988671-f8e8f267-55dc-44c2-944f-ce75fed773c7.png)

## Tirsdag 27.09 
Funnet ut hvordan vi kan endre utseende på huset slik at vi kan tilpasse det til et kjøpesenter. Nå må vi finne ut størrelsesforholdene 
![image](https://user-images.githubusercontent.com/112081507/192486573-a000cb86-703a-441c-aeb4-38696940a00d.png)

Tilla har laget bilde av kjøpesenteret på ipad og konvertert bildet til et occupancygrid som fungerer i matlab! Godt jobbbet! 
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

