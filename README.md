# ESP32-S3 8-channel logic analyzer — RX7500 / PAL / failover

Універсальний цифровий аналізатор для Waveshare ESP32-S3-Zero на чистому
ESP-IDF v6.0.1. Він зберігає робочий режим RX7500 і додає 8-канальне raw та
edge-event захоплення для PAL GPIO DAC, PAL field, LM1881/video failover і
довільних цифрових сигналів.

Arduino та PlatformIO не використовуються. Перед роботою прочитати
`PROJECT_CONTEXT.md`; точний wire protocol наведений у `PROTOCOL.md`.

## Безпека і піни

Усі входи ESP32 — **тільки 3,3 V digital input, no pull**. Спільна земля
обов'язкова. Не з'єднувати живлення плат. Не подавати аналоговий CVBS прямо на
GPIO: для нього потрібен comparator/sync separator/узгоджений level converter.

| Канал | ESP32-S3-Zero | RX7500 | PAL GPIO DAC | LM1881/failover |
|---:|---:|---|---|---|
| CH0 | GPIO4 | CLK | STM32 PA0 / VIDEO_D0 | REAL_CSYNC |
| CH1 | GPIO5 | DATA | STM32 PA1 / VIDEO_D1 | REAL_VSYNC |
| CH2 | GPIO6 | LE | STM32 PA2 / VIDEO_D2 | VIDEO_SEL |
| CH3 | GPIO7 | SPI/M | STM32 PA3 / VIDEO_D3 | fake sync reference |
| CH4 | GPIO8 | AUX4 | STM32 PA4 / VIDEO_D4 | AUX4 |
| CH5 | GPIO9 | AUX5 | STM32 PA5 / VIDEO_D5 | AUX5 |
| CH6 | GPIO10 | AUX6 | STM32 PB4 / VIDEO_SEL | AUX6 |
| CH7 | GPIO11 | AUX7 | AUX / CSYNC / VSYNC | AUX7 |

GPIO4…GPIO11 читаються одним доступом до `GPIO.in`; один raw sample — один
`uint8_t`. Офіційний pinout Waveshare підтверджує, що GPIO33…GPIO37 зайняті
Octal PSRAM і не виведені, WS2812 використовує GPIO21, USB Serial/JTAG —
GPIO19/GPIO20. GPIO4…GPIO11 не є boot strapping, USB, LED або in-package
flash/PSRAM лініями цієї плати.

## Профілі

- **RX7500 SPI** — raw 5 MHz, 8192 samples, CH0 rising, декодування 24-бітного
  `0x01 + frequency` кадру, CLK/DATA/LE/SPI-M timing і verdict. Кнопка
  `Legacy RX7500 R` запускає повністю сумісний v1 режим.
- **PEAK67 3-wire reverse engineering** — edge-event capture 20 ms із
  очікуванням першого transition обраного trigger до 10 s. Три проводи
  підключаються до CH0/GPIO4, CH1/GPIO5, CH2/GPIO6; декодер автоматично
  пропонує candidate mapping CLK/DATA/LE-CS за активністю й положенням
  переходів. Він не вгадує сам протокол: показує DATA на rising і
  falling CLK, MSB-first і LSB-first, кількість бітів/байти/ціле значення,
  clock HIGH/LOW/period, DATA setup/hold, імпульси третьої лінії та лише
  кандидат її ролі (`LE/latch` або `CS`).
- **PEAK67 power-on timing** — edge-event capture 1 s, trigger CH3/GPIO7
  rising. Монтаж: CH0=CLK, CH1=CS, CH2=DATA, CH3=3V3 sense. GUI шукає перше
  повне 32-CLK вікно або підтверджене PEAK67 CS LOW-вікно 150…250 µs і
  показує `3V3 rising → CS falling`, `→ first captured CLK rising` та
  `→ CS rising/commit`. Якщо edge ISR пропустив близькі CLK/DATA переходи,
  timing CS лишається доступним, але декодування слова явно позначається як
  неповне; для бітового підтвердження використовується окремий raw capture.
- **PAL GPIO DAC / line** — raw 5 MHz, CH0 falling; відновлення
  `DAC=CH0|(CH1<<1)…|(CH5<<5)`, step-графік, коди 0/19/41/63, автоматичний
  пошук лінії та вимір 64/4.7/5.8/52/1.5 µs. Реальна роздільна здатність при
  5 MHz — 200 ns, тому окремі 100 ns STM32 samples не відновлюються.
- **PAL field timing** — edge events, 45 ms; normal/equalizing/broad pulses,
  line/half-line/field/frame timing, vertical sequence і phase другого поля.
- **LM1881 / video failover** — edge events, 100 ms; CSYNC/VSYNC statistics,
  VIDEO_SEL transitions, loss/restore latency та fake-sync activity. Активний
  рівень VIDEO_SEL задається в GUI, він не вгадується.
- **Generic 8-channel** — довільні назви/видимість, raw або edge capture,
  configurable trigger і CSV без protocol-specific verdict.

## Acquisition limits

Raw capture: 100 kHz…5 MHz, 256…32768 samples. Максимальне 5 MHz вікно
триває 6.5536 ms; воно навмисно обмежене. Фактичну стабільність 5 MHz × 8
каналів ще треба перевірити на платі — успішний build не є hardware proof.

Edge capture: 1 ms…5 s, максимум 8192 подій, timestamp = CPU cycle counter
(фактична частота повертається в пакеті). ISR лише читає `GPIO.in` і записує
preallocated event; USB/CRC/аналіз виконуються поза ISR. Overflow/truncation і
trigger timeout явно повертаються у flags.

## Перше захоплення PEAK67

Підключення виконувати паралельно до трьох ліній між штатним контролером і
приймачем, не розриваючи їх:

```text
PEAK67 control line 0  -> ESP32 GPIO4 / CH0
PEAK67 control line 1  -> ESP32 GPIO5 / CH1
PEAK67 control line 2  -> ESP32 GPIO6 / CH2
PEAK67 GND             -> ESP32 GND
```

Спочатку мультиметром/осцилографом переконатися, що логічний HIGH не перевищує
3.3 V. Лінії ESP налаштовані inputs/no-pull і не повинні керувати PEAK67.

1. Вибрати profile `PEAK67 3-wire reverse engineering`.
2. `Auto` залишити вимкненим, натиснути `Capture`.
3. Протягом 10 секунд один раз змінити канал штатною кнопкою/енкодером.
4. Зберегти CSV і записати старий та новий канал/частоту.
5. Повторити окремо для переходів `канал N -> N+1`, `N+1 -> N` і для
   віддалених каналів. Не натискати кілька разів в одному першому capture.

Якщо з trigger CH0 отримано timeout або захоплено лише кінець команди,
повторити з trigger CH1, потім CH2. Після першого повного кадру GUI покаже,
який канал найбільш схожий на CLK. Автовизначення є лише кандидатом: його
треба підтвердити кількома різними командами.

Trigger на першому CLK гарантує захоплення команди, але не бачить точний lead
time третьої лінії до першого такту. Після першого визначення `LE` чи `CS`
можна окремо переставити trigger на `CH2 rising/falling`, щоб виміряти цей
інтервал. До порівняння кількох відомих каналів жоден з варіантів
MSB/LSB/rising/falling не вважається підтвердженим.

## Вимірювання затримки після подачі живлення

ESP32-S3 спочатку живиться окремо від USB і вже має бути підключений до GUI.
Живлення PEAK67 вмикається окремо після arm. Не можна живити ESP від тієї ж
лінії, яку він повинен побачити як trigger.

```text
PEAK67 CLK         -> ESP32 GPIO4 / CH0
PEAK67 CS          -> ESP32 GPIO5 / CH1
PEAK67 DATA        -> ESP32 GPIO6 / CH2
PEAK67 3V3 sense   -> 4.7…10 kΩ -> ESP32 GPIO7 / CH3
PEAK67 GND         -> ESP32 GND
```

На CH3 дозволено подавати лише перевірені 3,3 В, не 5 В. Вибрати профіль
`PEAK67 power-on timing`, натиснути `Capture`, переконатися у статусі ARMED і
після цього подати живлення на приймач. Захоплення триває 1 секунду після
CH3 rising. Значення відраховується від цифрового порогу GPIO7, тому це
затримка від детектування 3V3, а не від ідеального моменту 0 В.

Статична time-critical RAM: 32768 B raw + 65536 B edge events + 4096 B legacy
payload, разом близько 100 KiB без dynamic allocation у capture path.

## Запуск

1. Відкрити `C:\Espressif_progect\SKANER_LOG_ANALIZATOR` у VS Code.
2. Вибрати ESP-IDF v6.0.1 (`C:\esp\v6.0.1\esp-idf`).
3. Build/Flash через ESP-IDF extension. Закрити Serial Monitor після flash.
4. Запустити:

```powershell
py -3 tools\rx7500_analyzer.py
```

або `run_analyzer.bat`. У GUI вибрати COM, `Connect`, profile і `Capture`.
Новий `Apply / arm` надсилає versioned v2 command; `Legacy RX7500 R` надсилає
старий ASCII `R`.

Залежності й тести:

```powershell
py -3 -m pip install -r tools\requirements.txt
py -3 tools\test_protocol.py
```

## Структура

- `main/main.c` — 8-channel raw/edge engines, trigger, v1/v2 transport;
- `tools/rx7500_protocol.py` — streaming parser, packets, generic transitions;
- `tools/rx7500_analyzer.py` — Tkinter GUI та profile routing;
- `tools/pal_analyzer.py` — незалежний PAL line/field/failover analysis;
- `tools/test_protocol.py` — v1/v2, CRC, PAL synthetic regression;
- `PROTOCOL.md` — точний binary layout;
- `PROJECT_CONTEXT.md` — підтверджений стан і журнал.

## Поточна перевірка

- 17 Python protocol/PAL/PEAK67 tests: PASS.
- ESP-IDF v6.0.1 build: PASS; application `0x430e0`, 74% partition free.
- Нову версію навмисно не прошито без дозволу користувача.
- Нові 8-channel/PAL/PEAK67 edge режими ще не перевірені на реальному монтажі.
