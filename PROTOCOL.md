# ESP32-S3 analyzer binary protocol

USB Serial/JTAG є byte stream і може містити ESP-ROM boot text. Desktop parser
шукає capture magic `RX75`, підтримує fragmented reads, кілька пакетів за один
read, CRC errors і resynchronization. Усі integer — unsigned little-endian.

## Legacy capture v1

Збережено без зміни для RX7500. ASCII `R` озброює raw capture: GPIO4…GPIO7,
5 MHz, 8192 samples, CH0 rising. Header 28 B:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | `RX75` |
| 4 | 1 | version = 1 |
| 5 | 1 | flags: bit0 nibble packed, bit1 CH0 rising start |
| 6 | 2 | header size = 28 |
| 8 | 4 | capture number |
| 12 | 4 | sample rate Hz |
| 16 | 4 | sample count |
| 20 | 4 | payload size = ceil(sample_count/2) |
| 24 | 4 | payload CRC32 |

Payload: earlier sample у low nibble, next у high nibble. Bits 0…3 =
GPIO4…GPIO7.

## Configure-and-arm command v2

Command magic `LAC2`, рівно 36 B. CRC32 рахується по bytes 0…31.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | `LAC2` |
| 4 | 1 | version = 2 |
| 5 | 1 | command = 1 (`CONFIGURE_AND_ARM`) |
| 6 | 2 | size = 36 |
| 8 | 4 | request_id |
| 12 | 1 | acquisition: 1 raw, 2 edge |
| 13 | 1 | trigger channel 0…7; `0xFF` immediate |
| 14 | 1 | edge: 0 rising, 1 falling, 2 either, 3 immediate |
| 15 | 1 | flags = 0 (reserved) |
| 16 | 4 | requested sample rate Hz; edge mode = 0 |
| 20 | 4 | requested sample count; edge mode = 0 |
| 24 | 4 | requested duration µs; raw mode = 0 |
| 28 | 4 | trigger timeout ms; 0 disables timeout |
| 32 | 4 | command CRC32 over bytes 0…31 |

Firmware bounds:

- raw rate 100000…5000000 Hz and must divide actual CPU clock exactly;
- raw sample_count 256…32768;
- edge duration 1000…100000 µs;
- timeout 0…60000 ms;
- invalid command/version/CRC/parameters are ignored and never change capture.

## Capture v2 header

Header 56 B. It reports actual applied values, not merely requested values.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | `RX75` |
| 4 | 1 | version = 2 |
| 5 | 1 | packet flags |
| 6 | 2 | header size = 56 |
| 8 | 4 | capture number |
| 12 | 4 | request_id |
| 16 | 4 | timestamp frequency Hz (actual CPU clock) |
| 20 | 1 | acquisition: 1 raw, 2 edge |
| 21 | 1 | channel count = 8 |
| 22 | 1 | packing: 1 raw-u8, 2 edge-event8 |
| 23 | 1 | actual trigger channel |
| 24 | 1 | actual trigger edge |
| 25 | 1 | channel state before/at capture start |
| 26 | 2 | status = 0; reserved for explicit errors |
| 28 | 4 | actual sample rate; edge mode = 0 |
| 32 | 4 | item count: samples or events |
| 36 | 4 | actual capture duration in timestamp ticks |
| 40 | 4 | payload size bytes |
| 44 | 4 | lost edge event count |
| 48 | 4 | applied trigger timeout ms |
| 52 | 4 | payload CRC32 |

Packet flags:

- bit0 `TRIGGERED`;
- bit1 `TIMEOUT`;
- bit2 `OVERFLOW`;
- bit3 `TRUNCATED`;
- bits4…7 reserved.

Timeout packet may contain zero items and zero payload. CRC32 of empty payload
is zero.

### Raw payload (`packing=1`)

Payload size = item count. Кожен byte є одним одночасним `GPIO.in` sample:
bit0…bit7 = GPIO4…GPIO11. Sample time = `index / actual_sample_rate`.

### Edge payload (`packing=2`)

Payload size = item_count × 8. Event layout:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | timestamp ticks relative to capture start |
| 4 | 1 | complete 8-channel state after transition |
| 5 | 1 | changed mask |
| 6 | 2 | reserved = 0 |

Event time = `timestamp_ticks / timestamp_frequency`. Unsigned subtraction on
firmware makes the CPU cycle-counter wrap safe; maximum 100 ms capture is far
shorter than one 32-bit wrap at 240 MHz (~17.9 s). First immediate-trigger
event has timestamp 0 and changed mask 0.

## CRC and size validation

CRC is standard IEEE CRC-32, compatible with Python `zlib.crc32`. Parser first
validates version, header size, channel/packing/acquisition consistency,
item/payload limits and only then waits for or allocates payload. Maximum
accepted payload is 256 KiB.
