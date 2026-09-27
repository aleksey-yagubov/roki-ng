# Runtime-камера и синхронная IMU

Реализовано и проверено на голове без тела 26.09.2026. Runtime использует
libcamera напрямую, не Picamera2. Motherboard работает через USB ACM v2;
UART5 остаётся только загрузчику. Старый UART trace удалён, не оставлен заглушкой.

## Запуск

1. Supervisor запрещает одновременный direct-gst и runtime-захват.
2. Camera-worker подготавливает сенсор **1600x1300 RAW10**, ISP **800x650 BGR**,
   но ещё не запускает сенсор. AE/AWB отключены; exposure/gain заданы явно.
3. Motherboard-worker останавливает старый захват, задаёт фильтр периода
   стробов и выполняет 12 GetClock. Для каждого запроса измеряются времена
   отправки/ответа по CLOCK_BOOTTIME. Используется середина самого короткого
   окна; его полуширина должна быть не больше 1 мс.
4. STM сбрасывает strobe sequence командой StartStrobeCapture. Worker включает
   StartIMUStream(alignment), после подтверждений запускается камера.
5. Camera-worker сравнивает SensorTimestamp с пересчитанными в CLOCK_BOOTTIME
   метками **спада** строба. Это только определение соответствия счётчиков;
   сам STM подбирает IMU к **подъёму** строба.
6. Восемь пар с одинаковым Unicam−STM подтверждают соответствие. Допуск пары:
   минимум из четверти периода кадра и 3 мс. Метки с невалидной IMU или одним
   отсутствующим фронтом не участвуют. Переполнение TIM5 uint32 учитывается.
7. Supervisor переключает STM в normal, ждёт ACK и подтверждает synced
   camera-worker-у. Публикуется событие camera.synchronized.

Видео публикуется уже во время привязки. Сопоставлять IMU с кадрами до synced
нельзя. Если подтверждение не получено за 5 секунд, захват останавливается
с camera.fault; подстановка постоянного смещения не используется.

В normal дополнительные метки фронтов больше не нужны, STM не ждёт спада
для отправки готового результата. Записи на границе смены режима могут
завершаться не по порядку; worker учитывает sequence и убирает дубликаты.

## Shared memory

Два сервиса iceoryx2 с одним publisher и независимыми subscribers:

| Имя | Формат little endian | Содержание |
| --- | --- | --- |
| roki/camera/frame/v1 | `<IQIII` + пиксели | Unicam uint32, SensorTimestamp uint64 нс, width/height/stride uint32, BGR |
| roki/motherboard/imu/v1 | `<IQffffI` | STM sequence в uint32, timestamp Bosch uint64 нс, quaternion x/y/z/w float32, sensor ID uint32 |

STM sequence физически 16-битный; поле shared memory uint32 не увеличивает
его диапазон. Не предполагается непрерывный захват дольше оборота счётчика.
При обнаружении оборота worker останавливает захват, а не связывает новый
номер со старым кадром. Потребитель также не маскирует Unicam до 16 бит.

Timestamp кадра и timestamp Bosch принадлежат разным часам. Для сопоставления
нельзя просто вычитать их друг из друга. В IMU сервисе нет rise/fall timestamps;
они временно передаются по внутреннему IPC только при привязке.

Невалидные IMU не публикуются. Consumer получает пары по точному равенству:

```text
stm_sequence = unicam_sequence - unicam_minus_stm
```

SequenceJoiner до set_alignment игнорирует записи. При смене захвата он
инвалидируется и очищается; очереди потребителя также очищаются. После нового
synced устанавливается новое смещение. Нет подходящей IMU — нет пары;
ближайшее по номеру/последнее измерение не подставляется.

ISP DMA buffer копируется в iceoryx2 loan один раз, без padding. Это не
полный zero-copy от сенсора, но между процессами пиксели не сериализуются.
У каждого читателя bounded очередь; медленный читатель не блокирует камеру.

## Оператор и остановка

Команда camera.start требует MANUAL и действующую lease; camera.stop требует
lease, но не ограничена режимом MANUAL. Camera.status доступен наблюдателю.
Схема команд описана в
[OPERATOR_PROTOCOL_V1.md](OPERATOR_PROTOCOL_V1.md).

`camera.status.imu_sync.state`: disabled, aligning, matched, synced.
Matched означает найденное смещение, но ещё не подтверждённое переключение
STM. При synced доступны unicam_minus_stm, pairs, max_residual_ns,
clock_uncertainty_ns. Остаток timestamp — диагностическая оценка привязки,
не измерение абсолютной точности ориентации IMU.

`with_imu:false` явно выключает строб/IMU stream. Direct-gst также не включает
их. Остановка camera-worker сопровождается StopStrobeCapture; это выключает
stream и очищает историю STM. Сам сенсор IMU продолжает измерения.
Ошибка ACM или переполнение нативной очереди вызывает остановку синхронного
захвата. Ошибка отсутствующего тела этого не делает.

## Проверки на голове

Запуск из `/opt/roki-ng`, без другого runtime:

```sh
python3 tools/check_camera_imu.py
python3 tools/check_camera_imu.py --30fps
python3 tools/check_camera_imu.py --camera-only
python3 tools/check_camera_imu.py --probe-body
```

Это head-only тест без движений. Каждый запуск делает два последовательных
захвата. Consumer — отдельный процесс, действительно читающий кадры и IMU
через iceoryx2. После synced измеряется около 8 секунд.

- 60 FPS: смещения 5 и 4, соответственно 478 и 479 точных пар.
- 30 FPS: смещение 2, по 239 точных пар в обоих запусках. Первые две невалидные IMU из-за
  стартового периода сенсора пропускаются, не подменяются нулевыми измерениями.
- Camera-only: 480 кадров в каждом окне, IMU-записей 0.
- С probes отсутствующего тела: смещения 5 и 4, 479 и 478 пар. Состояние
  motherboard degraded, ошибка BodyTimeout; поток IMU продолжает работать.
- Во всех этих окнах unmatched_evicted=0. Различие количества кадров и IMU
  на границах измерения допустимо. Результаты не доказывают бесконечную работу
  без потерь или механическую корректность движений.

Логи: `out/motherboard-acm-v2-20260926/acm-workers-*.log` в общем workspace;
на голове `/root/acm-workers-*.log`. Firmware/library тесты отдельно:
[ACM_V2_HEAD_TEST.md](https://github.com/aleksey-yagubov/roki-mb-firmware/blob/master/ACM_V2_HEAD_TEST.md).

Дополнительно прошёл захват 300 секунд с probes отключённого тела: 17986 кадров
и 17986 IMU в окне consumer, 17984 точные пары, unmatched_evicted=0. На границах
измерения по одной неполной паре допустимо. IMU invalid/errors=0.

При SIGKILL camera-worker surviving motherboard выключает strobe capture.
При SIGKILL motherboard-worker supervisor останавливает камеру, но отправить
STOP в STM через умершего владельца уже невозможно. У firmware пока нет
watchdog владельца USB: capture_active остаётся true без новых стробов.
Следующая инициализация motherboard-worker явно выключает старый захват;
тест check_capture_faults.py после завершения процессов делает такую очистку.
Это ограничение не выдаётся за исправленное.

Runtime-encoder и базовый LAB-consumer уже добавлены:
[RUNTIME_VIDEO_AND_DETECTION.md](RUNTIME_VIDEO_AND_DETECTION.md).
Следующие этапы: семантические детекторы, локализация и игра. Не объявляем
готовыми автономную игру, OSD или использование синхронной IMU локализацией.
