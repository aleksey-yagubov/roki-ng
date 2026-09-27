# Протокол оператора и передача медиа

Это целевая архитектура всех этапов. Точная спецификация уже реализованного
набора `manual-1` находится в [OPERATOR_PROTOCOL_V1.md](OPERATOR_PROTOCOL_V1.md).
Нереализованные группы не объявляются capabilities; в частности live bitrate,
OSD, servo telemetry и bulk transfer пока недоступны. Runtime video уже реализовано.

## Фиксированная модель подключения

Discovery-протокола нет. Робот не использует broadcast, multicast, mDNS и
никаким другим способом не объявляет о своём присутствии.

Оператор вводит IP робота и отправляет `hello` прямо на фиксированный UDP
control port, настроенный на обеих сторонах; начальное значение — `8093`.
Робот отвечает только source endpoint валидного пакета и не отправляет
телеметрию до явной подписки.

Обычно каждая сторона использует один UDP socket для MessagePack-команд,
ответов, событий, телеметрии и коротких OSD metadata. Только видео передаётся
через согласованный RTP/UDP port. OSD ограничен бюджетом MessagePack и не
содержит пикселей; отдельного OSD RTP-потока нет.

## Конверт datagram

Рабочий datagram должен укладываться в безопасный лимит UDP payload, изначально
1200 байт. Это исключает IP fragmentation в обычных Ethernet/Wi-Fi сетях.

```text
{
  "v": 1,
  "kind": "hello" | "welcome" | "request" | "response" | "event" | "sample",
  "session": 64-битный ID сессии или 0,
  "token": 64-битный token сессии или 0,
  "id": 64-битный ID запроса или 0,
  "sequence": 64-битный sequence события/потока или 0,
  "robot_mono_ns": monotonic timestamp робота или 0,
  "op": "video.create",
  "body": {...}
}
```

## MTU и запрет IP fragmentation

Все UDP sockets протокола создаются с запретом IP fragmentation. На Linux для
IPv4 используется `IP_MTU_DISCOVER=IP_PMTUDISC_DO`, для IPv6 —
`IPV6_MTU_DISCOVER=IPV6_PMTUDISC_DO`; на других платформах применяется
эквивалентная socket option. Правило распространяется на:

- MessagePack control, response, event и datastream packets;
- video RTP;
- RTCP, если он включён.

Превышение path MTU должно завершать отправку с `EMSGSIZE` и увеличивать
диагностический counter. Реализация не включает fragmentation в качестве
fallback. MessagePack producer проверяет предел 1200 байт до `send`.

Начальный RTP packet MTU для прямого Ethernet/Wi-Fi равен 1400 байт и включает
RTP header и header extensions, но не внешние UDP/IP headers. Значение является
параметром media session. Для маршрута через WireGuard с interface MTU 1420
используется 1360 байт. `stream-worker` создаёт и настраивает UDP `GSocket`,
передаёт его в `udpsink` через свойство `socket` и не позволяет GStreamer
создать socket с другими параметрами.

Запрос и ответ используют `id`, событие и sample — `sequence`. Ответ
содержит `result` либо структурированную ошибку:

```text
{
  "error": {
    "code": "camera_busy",
    "message": "camera is owned by autonomous runtime",
    "retryable": false,
    "details": {...}
  }
}
```

Неизвестная операция отклоняется. Неизвестные поля известной версии протокола
игнорируются, но смысл существующего поля внутри одной версии не меняется.

## Установление соединения и сессии

`hello` является прямым handshake, а не discovery:

```text
{
  "v": 1,
  "kind": "hello",
  "session": 0,
  "id": 1,
  "body": {
    "versions": [1],
    "client_name": "roki-operator",
    "client_instance": "random-128-bit-id"
  }
}
```

`welcome` выбирает версию и возвращает:

- случайные session ID и session token;
- `robot_id` и случайный `boot_id`, меняющийся при каждом запуске supervisor;
- монотонное время робота и интервал heartbeat;
- текущее верхнеуровневое состояние;
- revision/hash capabilities и limits протокола.

Сессия привязана к source IP/port. Каждый пакет после handshake содержит
полученный token. При смене локального socket клиент выполняет новый handshake.
Token защищает от случайного смешения сессий, но не является криптографической
аутентификацией. В недоверенной сети опасным операциям нужен PSK-механизм либо
защищённая сеть.

Клиент отправляет `session.heartbeat` с согласованным интервалом. Истечение
сессии освобождает её управляющую lease и останавливает только созданные ей
ресурсы: видеокодеры оператора и подписки на datastream. Автономная игра
не останавливается.

## Гарантии доставки

Семантика UDP задаётся для каждого класса операций:

- запрос, изменяющий состояние, идемпотентен по `(session, id)`;
- при отсутствии ответа клиент повторяет запрос с тем же ID;
- робот ограниченное время кэширует готовый ответ и не выполняет duplicate;
- непрерывные manual drive, telemetry, OSD и обычные события не повторяются;
- разрывы sequence видимы и исправляются запросом нового snapshot, а не
  воспроизведением устаревших live samples;
- timeout запроса не отменяет уже запущенный job.

Control responses и safety events имеют высший outbound priority. Telemetry и
OSD используют заменяемые latest-only queues. Медленный клиент теряет samples,
но не задерживает ответ и не вызывает неограниченный рост памяти робота.

## Управляющая lease и режимы

Наблюдать могут несколько сессий. Управляющая lease принадлежит ровно одной.
`control.acquire` возвращает `lease_epoch`; каждая изменяющая состояние
команда содержит эту epoch и отклоняется после её отзыва.

Lease и режим робота — разные сущности. Lease разрешает отправлять команды, а
state machine решает, допустима ли команда сейчас. Потеря lease во время
автономной игры не меняет game state. В manual drive она запускает настроенную
плавную deadman-остановку.

Группы публичного API:

- `system.*`: identity, capabilities, health, logs, reboot и shutdown;
- `log.*`: sources, subscribe, update, unsubscribe и snapshot;
- `session.*`, `control.*`: heartbeat и управление lease;
- `mode.*`: idle, manual, calibration, game, pause и recovery;
- `motion.*`: drive, head, pose, slot, graceful stop и hard stop;
- `game.*`: role, side, start pose, start, pause, continue и target override;
- `camera.*`: capabilities sensor/output и текущий owner/state;
- `video.*`: create, start, update, stop и destroy encoded streams;
- `data.*`: список topics, subscribe, update, unsubscribe и snapshot;
- `osd.*`: каталог video OSD layers и subscriptions;
- `servo.*`: inventory, telemetry, recovery и guarded parameters;
- `params.*`: keys, describe, get и set типизированных параметров;
- `calibration.*`: list, describe, start, status, input и stop;
- `test.*`: отдельные аппаратные и двигательные проверки, не изменяющие
  параметры автоматически;
- `strategy.*`: данные стратегии и временные runtime overrides;
- `head_display.*`: физический экран робота.

Длительная операция сразу возвращает `job_id`. Её жизненный цикл публикуется
через `job.progress`, `job.completed` и `job.failed`. `job.cancel`
запрашивает bounded cancellation. Только явный hard stop может оборвать штатное
завершение движения и потребовать recovery.

## Логи времени выполнения

Supervisor всегда собирает bounded runtime log history независимо от настройки
stdout. После подключения операторское приложение по умолчанию автоматически
вызывает `log.subscribe`. Запрос задаёт минимальный level, список sources и
количество последних записей для начального replay.

Log sample передаётся MessagePack batch-ами через основной control socket:

```text
{
  "subscription": 12,
  "sequence": 481,
  "records": [
    {
      "record_sequence": 9912,
      "monotonic_ns": 1234567890,
      "level": "INFO",
      "source": "camera-worker/libcamera",
      "message": "camera started"
    }
  ],
  "dropped": 0
}
```

`log.update` и `log.unsubscribe` меняют фильтр или прекращают live-передачу;
`log.snapshot` возвращает bounded последние записи без постоянной подписки.
Batch всегда укладывается в MessagePack datagram budget 1200 байт. Длинные
строки заранее ограничиваются, а многострочные traceback передаются
последовательностью records.

Log traffic имеет приоритет ниже responses, safety events и управляющих команд.
При перегрузке отбрасываются DEBUG/INFO batches и увеличивается `dropped`;
управление роботом не задерживается. ERROR в log stream не заменяет
структурированный health/fault event.

Клиент показывает logs в отдельной Qt-модели с фильтром source/level, поиском,
pause и опциональным autoscroll. Pause останавливает только обновление виджета,
но не работу робота; после resume модель продолжает с новых records и явно
показывает обнаруженный sequence gap.

## Потоки данных

Datastream запрашивается явно и состоит из topics. `data.list` возвращает
стабильное имя, revision schema, семантику, допустимые частоты и требуемый режим
каждого topic.

Виды topic:

- `state`: полный последний snapshot; старые samples можно заменять;
- `series`: упорядоченные измерения, по возможности собранные в один datagram;
- `event`: дискретные переходы, например падение или отказ worker-а.

Начальные topics:

- `system.workers`, `system.resources`;
- `motion.state`, `motion.odometry`, `motion.fall`;
- `imu.head`, `imu.body`, `power.battery`;
- `servo.health` и необязательные подробности отдельных серв;
- `camera.state`, `camera.timing`;
- `vision.objects`, `vision.timing`;
- `localization.state`, particles и confidence;
- `game.state`, выбранная цель и planned action;
- `neural.state`, время inference.

Пример подписки:

```text
data.subscribe {
  "topics": [
    {"name": "localization.state", "rate_hz": 10},
    {"name": "servo.health", "rate_hz": 2}
  ]
}
```

Ответ содержит `subscription_id`, компактные числовые IDs topics,
применённые rates и schemas. В одном sample можно собрать несколько записей:

```text
{
  "subscription": 31,
  "sequence": 92,
  "samples": [
    {
      "topic": 4,
      "source_sequence": 1881,
      "source_mono_ns": 99112233,
      "valid": true,
      "age_ms": 3,
      "data": {...}
    }
  ]
}
```

Первый пакет после подписки является полным snapshot. Клиент обнаруживает
пропуски по subscription sequence. Rates являются верхним пределом: неизменное
состояние можно не отправлять, а при перегрузке фактическая частота снижается.

Локализация и game state передаются именно здесь. Карта поля на компьютере
строится из `localization.state`, `vision.objects`, `game.state` и motion
topics и не требует видеопотока или OSD.

## Жизненный цикл видео

Видео не запускается от `hello`, `camera.capabilities`, входа в manual mode
или подписки на telemetry.

Жизненный цикл состоит из двух фаз:

1. `video.create` проверяет параметры, создаёт stream ID и возвращает
   применённые RTP caps, не начиная capture и передачу.
2. Оператор создаёт и переводит receiving pipeline в PAUSED.
3. `video.start` при необходимости запускает camera/backend и начинает RTP.
4. `video.update` изменяет поддерживаемые live properties, например bitrate.
5. `video.stop` прекращает передачу, сохраняя определение stream.
6. `video.destroy` освобождает encoder и ресурсы сессии.

Пример `video.create`:

```text
{
  "backend": "runtime" | "direct-gst" | "auto",
  "sensor": {
    "mode_id": "1600x1300-RG10",
    "fps": 0
  },
  "output": {
    "width": 800,
    "height": 650,
    "fps": 0
  },
  "codec": {
    "name": "h264" | "jpeg",
    "bitrate": 2000000,
    "profile": "baseline",
    "keyframe_interval": 30
  },
  "destination": {
    "rtp_port": 5004,
    "rtcp_port": 5005
  }
}
```

По умолчанию destination IP совпадает с source IP сессии и не может быть
изменён, чтобы робот не стал источником UDP reflection.

`direct-gst` использует `libcamerasrc` для ручного наблюдения, когда
автономному коду не нужны пиксели. `runtime` кодирует кадры работающего
libcamera runtime и сохраняет frame identity, используемый detection и
localization. GStreamer pipelines обоих backend-ов принадлежат stream-worker;
camera-worker владеет камерой только в runtime backend.

`direct-gst` не использует frame-synchronised IMU и не управляет strobe drain
ради видеопотока. Это backend только для ручного управления и наблюдения:
обычные движения продолжают работать, а независимый IMU datastream запускается
только по явной подписке и по умолчанию неактивен. Detection, localization и
точный OSD для этого видео недоступны.

Bitrate можно менять live. Изменение codec format или dimensions может
перезапустить только encoder. Смена sensor mode или camera backend закрывает
старые video streams и создаёт новые stream IDs. При входе в `runtime`
camera-derived rings создаются заново, а camera/STM counters и strobe
containers согласованно сбрасываются. Запуск `direct-gst` не требует
синхронизации с IMU. Ответ всегда сообщает применённые настройки и
перезапущенные ресурсы.

Каждый работающий video stream сообщает:

- stream ID, RTP SSRC, payload type и clock rate;
- codec caps для receiving pipeline;
- output dimensions и измеренный frame rate;
- URI/ID RTP header extension с 32-битным `UnicamSequence`;
- доступные MessagePack OSD layers, если OSD поддерживается;
- `video_stream_id` и преобразование source/output;
- счётчики packets, frames и drops;
- доступность точной синхронизации OSD.

H.264 настраивается на low latency без B-frame reordering. RTCP при наличии
используется для receiver reports и диагностики, но управление остаётся в
MessagePack.

## OSD

`osd.*` управляет подписками на короткие примитивы поверх camera streams. Сами
обновления передаются MessagePack через control socket, а кадры несут
`UnicamSequence` в RTP header extension. Клиент рисует overlay средствами Qt 6
без OpenCV. Локализация, объекты в координатах поля, игровые цели, IMU и
состояние серв остаются topics `data.*`. Полная модель описана в
[DISPLAY_PROTOCOL.md](DISPLAY_PROTOCOL.md).

## Большие объекты

Архивы логов, диагностические snapshots и таблицы стратегии могут превышать
лимит рабочего datagram. Для них используется transfer прикладного уровня:

1. запрос создаёт transfer с total size, SHA-256 и chunk size;
2. нумерованные binary chunks укладываются в лимит UDP payload;
3. приёмник сообщает ranges отсутствующих chunks;
4. повторяются только отсутствующие chunks;
5. завершение требует совпадения размера и hash.

Видео, телеметрия и OSD этот надёжный механизм не используют. Большие
transfers имеют строгий bandwidth budget и приоритет ниже control traffic.
