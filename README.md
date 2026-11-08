# Robot rozwiązujący labirynt: symulator

Program robota jest pisany w C (`firmware/`) i woła sprzęt wyłącznie przez `firmware/include/hal.h`.
Symulator (`sim/`, Python + pygame) uruchamia go jako proces potomny i odpowiada na wywołania HAL
przez stdin/stdout. Ten sam kod C kompiluje się z `hal/arduino/hal_arduino.cpp` na prawdziwe Arduino Uno.

## Uruchomienie

Linux:

```bash
./run.sh                                  # maps/demo.json
./run.sh --map maps/moja.json --new 40x40 # nowa pusta mapa 40x40 komórek po 10 cm
./run.sh --map maps/moja.json --new 40x40 --cell 0.05
```

Windows (Windows 11, okno pokazuje WSLg):

1. Zainstaluj WSL2 z Ubuntu (`wsl --install`) i Docker Desktop z włączoną integracją WSL.
2. Sklonuj repo **wewnątrz WSL** (np. `~/projekt`, nie `C:\...`), w terminalu Ubuntu uruchom `./run.sh`.

Pierwsze uruchomienie buduje obraz Dockera (kilka minut).

## Sterowanie

| Tryb | Klawisze |
|------|----------|
| symulacja | `SPACE` pauza, `R` reset, `B` przebuduj kod C i reset, `+`/`-` tempo, `M` maks. tempo, `T` czyść ślad, `C` stożek sonaru, `E` edytor |
| edytor | LPM ściana, PPM gumka, `[` `]` pędzel, `S` start pod kursorem, kółko obraca start, `G` cel, `X` czyść, `Ctrl+S` zapis |

Po zmianie kodu w `firmware/src/` wystarczy nacisnąć `B` w oknie symulatora.

## Model sprzętu

Wszystkie parametry są w `sim/hardware.py`, a to, co o sobie "wie" robot, w `firmware/include/robot_config.h`.

- Silniki DC 1:48, 200 rpm przy 6 V, TB6612FNG (spadek 0,3 V), koła 65 mm, rozstaw 13 cm: przy PWM 255 jest ok. 0,65 m/s.
- Serwo SG90: 600°/s (0,1 s/60°), 0° = prawo, 90° = przód, 180° = lewo.
- HC-SR04: 2-200 cm, stożek 15°, szum, brak echa przy kącie padania > 40°, pomiar trwa tyle, co prawdziwe echo (timeout 38 ms), ostrzeżenie przy pomiarach częstszych niż co 60 ms.
- Kolizje: robot nie wjeżdża w ścianę, ruch jest blokowany (robot się klinuje).
- Czas: zegarem jest symulator. Każde wywołanie HAL kosztuje 20 µs, `hal_delay_ms` i pomiar trwają swój czas, obliczenia między wywołaniami nie zajmują czasu.

## Firmware

```bash
docker compose run --rm sim make -C firmware avr-check  # kompilacja pod ATmega328P + zużycie RAM
docker compose run --rm sim make -C firmware arduino    # szkic w firmware/build/arduino/robot/
```

Piny Arduino są w `firmware/hal/arduino/hal_arduino_pins.h` (HAL Arduino nie był jeszcze testowany na sprzęcie).
