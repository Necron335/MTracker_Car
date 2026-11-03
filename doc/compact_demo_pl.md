# Krótka demonstracja skrętów z RViz

Uruchamia **8 par przód–tył**, czyli 16 przejazdów po około 30 cm.
Kąt skrętu zmienia się w połowie każdego przejazdu, **bez zatrzymywania**.
Postój występuje tylko przed zmianą kierunku jazdy i na końcu demonstracji.
Używa `AckermannDriveStamped`, skalibrowanego kąta
`±0.26 rad` i prędkości zadanej **0.25 m/s**. Prędkość rzeczywista może być
mniejsza, ponieważ odcinki są krótkie. Nie ma pełnego okręgu ani obrotu w miejscu.

W przód kolejność skrętu to lewo–prawo, a przy cofaniu prawo–lewo,
żeby w modelu kinematycznym wracać po tym samym śladzie.
Program zaczyna zatrzymywanie wcześniej, zależnie od zmierzonej prędkości.
Plan to około **4.8 m łącznie** i około **30–35 s** pracy.
Przed zmianą kierunku potwierdza niską prędkość przez co najmniej 0.3 s;
postój trwa co najmniej 0.7 s. Po ośmiu parach demonstracja kończy się.

## Limity

Żądane **0.5 m oznacza oddalenie od pozycji startowej**, a nie sumę wszystkich
przejazdów. Przy 0.25 m/s droga 0.5 m trwałaby około 2 s.
Monitoruje całkowitą drogę, również podczas postojów, i odległość od startu.
Przy **0.40 m oddalenia od startu** wysyła stop, pozostawiając zapas do 0.5 m.
Osobny limit sumarycznej drogi wynosi 6 m, a czasu całego pokazu 60 s.
Po utracie odometrii na 0.10 s przerywa ruch. Driver ma timeout komend 0.15 s.
Przekroczenie prędkości 0.45 m/s, skok odometrii, przeciwny kierunek,
brak zatrzymania lub przekroczenie czasu też przerywa demonstrację.

Są to ograniczenia według odometrii środka tylnej osi, nie gwarancja fizycznej
granicy obrysu pojazdu. Poślizg, błąd odometrii i droga hamowania wpływają na
rzeczywisty ruch. Schemat robota w RViz nie jest dokładnym modelem wymiarowym.
Program nie wykrywa przeszkód. Przed jazdą sprawdź poprawność kalibracji skrętu.

## Uruchomienie

Zakończ inne źródła jazdy i poprzedni sterownik. W pierwszym terminalu:

```bash
cd /home/student/ros2_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mtrackercar --symlink-install
source install/setup.bash
ros2 launch mtrackercar compact_demo.launch.py enable_motion:=true
```

Launch otwiera RViz i uruchamia własny sterownik. **Nie rozpoczyna jazdy**.
W drugim terminalu załaduj te same środowiska i uruchom sekwencję:

```bash
ros2 service call /compact_demo/start std_srvs/srv/Trigger '{}'
ros2 topic echo /compact_demo/status
```

Anulowanie:

```bash
ros2 service call /compact_demo/cancel std_srvs/srv/Trigger '{}'
```

Kolejny przebieg wymaga restartu modułu. Bez autoryzacji ruchu użyj
`enable_motion:=false` (domyślnie) — widok będzie działać, ale usługa start
odmówi rozpoczęcia jazdy. `rviz:=false` wyłącza okno. `start_driver:=false`
pozwala użyć istniejącego sterownika; musi przyjmować `drive_command`,
publikować `/wheel/odom` dla `odom`/`base_link`, mieć krótki timeout komend
i nie współdzielić sterowania z innymi publisherami.

## Widok RViz

- siatka co 5 cm, widok z góry zakotwiczony przy pozycji startowej;
- zielony ślad zmierzonego ruchu `/compact_demo/trajectory`;
- osie pokazujące zmierzoną orientację `base_link`;
- schemat nadwozia i kół; pomarańczowe przednie koła pokazują **zadany** skręt,
  a nie niezależny pomiar ich kąta;
- żółty okrąg o promieniu 0.4 m oznacza limit odległości od startu.

Przy odtwarzaniu tego samego łuku do tyłu ślady mogą się nakładać.
Ślad, status i RViz nie są automatycznie zapisywane do raportu; do rejestracji
użyj rosbag. Domyślne demonstracje okręgu pozostają osobnymi launchami
(`circle_demo`, `feedback_circle`), z promieniem 1 m.

## Brak ramek w RViz lub brak usługi start

Oczekiwane transformacje to `odom → base_link` (z drivera) oraz
`odom → compact_demo_start` (z modułu demonstracji). RViz ma `Fixed Frame: odom`.
Druga ramka pojawia się po otrzymaniu poprawnej odometrii; nie jest tworzona
na podstawie domyślnej pozycji. Nie dodawaj sztucznego stałego `odom → base_link`,
bo zamaskuje brak rzeczywistego ruchu.

Jeżeli pozostał tylko RViz i moduł trajektorii, sprawdź błędy startu drivera
i `compact_demo`. Po dodaniu zależności Ackermann starszy terminal może nie
mieć jej w ścieżkach bibliotek. Zakończ poprzedni launch przez `Ctrl+C`,
wykonaj oba polecenia `source` z sekcji uruchomienia w terminalu launch i
uruchom ponownie. Aktualny launch sprawdza te zależności przed otwarciem okna
i kończy pozostałe procesy, gdy driver lub demo się wyłączy.

Diagnostyka w drugim terminalu z załadowanym środowiskiem:

```bash
ros2 node list
ros2 topic echo /wheel/odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo odom compact_demo_start
```

Do działania potrzebne są nody `/mtrackercar` i `/compact_demo`, a do publikacji
rzeczywistej odometrii — potwierdzona komunikacja z robotem. Samo okno RViz
nie oznacza, że sterownik działa. Przy `start_driver:=false` źródło odometrii
i TF musi zapewnić już działający driver.
