# Архитектура процессов

> Обновление 24.09.2026: транспорт кадров, IMU и результатов будет основан на
> iceoryx2, а не собственных rings. Описание memfd/seqlock ниже историческое.
> См. [IPC_TRANSPORT_DECISION.md](IPC_TRANSPORT_DECISION.md).

## Граф компонентов

```text
dinit
└── roki-supervisor
    ├── motherboard-worker
    ├── camera-worker
    ├── stream-worker
    ├── detection-worker
    ├── localization-worker
    ├── game-worker
    ├── neural-worker             (необязательный)
    └── head-display-worker       (необязательный физический экран)
```

Это новая runtime-система, а не обвязка старой монолитной программы. Камера,
детекция, локализация и игровая логика с первой реализации являются отдельными
процессами. Благодаря этому у каждого тяжёлого Python-компонента собственный
GIL, а процессы можно независимо перезапускать, профилировать и изолировать при
ошибках.

Worker-ы не ищут друг друга. Supervisor до запуска создаёт весь граф локальных
каналов, описанный в [INTERNAL_IPC.md](INTERNAL_IPC.md). Между процессами
передаются неизменяемые типизированные записи; общего изменяемого объекта
`glob` нет.

Текущая реализация использует основной UART и прежний `Roki.Motherboard` API.
Решением от 26.09.2026 следующий транспорт — USB ACM для всего runtime,
без отдельного второго UART и отдельного `imu-worker`. Ниже описана целевая
архитектура; переход ещё не реализован. Контракт:
[MOTHERBOARD_PROTOCOL_V2.md](MOTHERBOARD_PROTOCOL_V2.md).

## Ответственность компонентов

### Supervisor

- Владеет единственным фиксированным UDP/MessagePack endpoint, сессиями клиентов
  и управляющей lease. Broadcast и discovery не используются.
- Читает четыре кнопки меню через evdev; `BTN_RST` остаётся в platform-сервисе
  выключения.
- Владеет верхнеуровневой state machine, запускает и контролирует worker-ы.
- Распределяет эксклюзивные ресурсы и отклоняет несовместимые запросы.
- Организует запуск и восстановление аппаратуры.
- Маршрутизирует команды и публикует состояние worker-ов, но не обрабатывает
  кадры.
- Агрегирует runtime logs всех worker-ов, хранит bounded history, отправляет её
  подписанному оператору и опционально зеркалирует в собственный stdout.
- Никогда не владеет объектами libcamera, GStreamer, OpenCV или Roki.

Запуск motherboard является операцией supervisor, хотя runtime-доступ выполняет
worker:

1. Остановить прежний motherboard-worker и убедиться, что его порт закрыт.
2. Получить reset GPIO STM32H743 и выполнить детерминированный полный reset.
3. Открыть `/dev/ttyAMA5` со скоростью bootloader.
4. Определить плату и отправить `CMD:GET_CONNECTION_STATE`, затем
   `CMD:START_FW`.
5. Закрыть serial-объект bootloader.
6. Запустить `motherboard-worker`: найти ACM по USB product `Roki motherboard`,
   открыть его и проверить версию runtime-протокола.
7. Дождаться self-test и события `motherboard.ready`.

Serial fd и Python-объекты между процессами не передаются. Supervisor передаёт
только конфигурацию и состояние готовности. Полный reset во время работы сначала
останавливает всех пользователей STM, затем повторяет эту последовательность.

### Motherboard-worker

STM32H743 является gateway не только к телу, но также к IMU головы и контейнерам
camera strobe.

- Единолично владеет runtime ACM, `Roki.Motherboard`, `Roki.Rcb4` и
  `Roki.Zubr`.
- Выполняет движения тела/головы и публикует состояние движения.
- По требованию читает последние body/head IMU. Для синхронного runtime camera
  pipeline включает захват стробов и IMU stream, получает готовые записи
  асинхронно по ACM, без `GetIMUFrame(sequence)` для каждого кадра.
- Настраивает strobe filter и управляет явными start/stop захвата.
- Опрашивает постоянный слот контроллера с телеметрией серв и выполняет
  защищённые сервисные команды. Другие процессы к mailbox Zubr не обращаются.
- Публикует компактные IMU и body state в shared memory. Каждая strobe IMU
  запись содержит 16-битный `stm_sequence`.
- Сериализует операции тела одним I/O scheduler. Один нативный приёмник ACM
  разбирает RPC-ответы и IMU records; ожидание ответа тела не блокирует приём IMU.
  Телеметрия не должна задерживать urgent motion stop.
- Различает плавную остановку походки и жёсткий сброс очереди.

Motherboard-worker не классифицирует падения и не решает, когда вставать.
Он публикует IMU и состояние исполнения движений. Оценка состояния робота
(наклонён/упал/лежит) находится в другой логической части, а режим игры или
оператор решает, запускать ли вставание. Само движение вставания выполняет
motherboard-worker по явной команде. Обратная связь IMU внутри конкретного
движения (например, удержание курса) остаётся допустимой.

Длительное движение не должно занимать command loop целиком. Временно
блокирующие функции запускаются как jobs, но целевой вариант продвигает ходьбу
и длинные позы ограниченными шагами, чтобы между ними обрабатывать stop и IMU.
STM хранит историю strobe-записей, поэтому небольшая задержка не меняет
соответствие IMU кадру.

### IMU stream через общий ACM

Motherboard-worker является единственным producer-ом IMU history в iceoryx2.
Его приёмник разделяет RPC responses по request_id и асинхронные IMUFrame по
типу сообщения. Второй читатель того же ACM не допускается.

Захват стробов и отправка IMU по умолчанию выключены. До старта синхронного
camera pipeline worker выполняет StartStrobeCapture и StartIMUStream,
дожидаясь подтверждений. STM отправляет запись после завершения сопоставления,
а не непосредственно из обработчика фронта. Порядок остановки и границы
очистки описаны в [MOTHERBOARD_PROTOCOL_V2.md](MOTHERBOARD_PROTOCOL_V2.md).

Для direct-gst/libcamerasrc оба режима выключены. Сам сенсор IMU продолжает
работать, GetIMULatest доступен независимо от камеры. UART5 CM4 используется
supervisor-ом только для загрузчика; STM UART8 к телу остаётся без изменений.

### Camera-worker

- Единолично владеет камерой при runtime-захвате через libcamera.
- Один раз публикует каждый кадр в общий bounded shared-memory FrameRing для
  всех потребителей пикселей.
- Публикует descriptor с `UnicamSequence`, временем сенсора, размером, форматом
  пикселей и номером slot.
- Не запускает capture, пока он не нужен игре, калибровке или явному запросу
  оператора.

### Stream-worker

- Владеет GStreamer encoder, RTP payloader, video sessions и UDP media sockets.
- Для backend `runtime` читает общий FrameRing camera-worker-а и не владеет
  камерой.
- Для backend `direct-gst` получает camera lease и запускает `libcamerasrc`,
  только когда runtime capture полностью остановлен.
- Для backend `runtime` добавляет `UnicamSequence` в RTP header extension
  каждого camera frame. `direct-gst` не обещает точную идентификацию кадра.
- Не передаёт отдельный OSD RTP. Примитивы идут через MessagePack; предлагаемый
  маршрут через supervisor описан в DISPLAY_PROTOCOL.md и ещё не реализован.
- Обрабатывает GStreamer `ERROR`/`EOS` внутри конкретной stream session:
  переводит её в `FAILED`, освобождает pipeline и публикует ошибку, не завершая
  другие worker-ы.
- Не ждёт OSD перед отправкой видео и не добавляет latency основному потоку.

Одновременно камерой может владеть только один backend. Выход из `runtime`
останавливает camera-derived pipeline и уничтожает его rings. При входе в
`runtime` камера и STM проходят согласованный сброс счётчиков и получают новый
чистый комплект rings. Запуск `direct-gst` сам по себе strobe-контейнеры не
сбрасывает.

`direct-gst/libcamerasrc` является video-only backend ручного режима. Он не
запускает detection/localization, не сопоставляет кадры с IMU и не управляет
strobe drain. Motherboard-worker продолжает обслуживать движения. Независимая
IMU telemetry в этом режиме опциональна, запускается только явной подпиской и
по умолчанию неактивна. При возврате в `runtime` выполняется полный reset
strobe containers и новая синхронизация sequence.

### Detection-worker

- Читает кадры из shared memory.
- Выполняет цветовую детекцию, линии, ворота, ArUco/AprilTag и другие
  классические detector-ы.
- Использует latest-only scheduling: при отставании выбрасывает старые
  необработанные кадры вместо накопления задержки.
- За одну итерацию запускает настроенный набор классических detector-ов над
  одним snapshot кадра и публикует общий observation batch. Detector-ы с более
  низкой требуемой частотой запускаются по собственному period, но каждый раз
  также используют новейший кадр.
- Возвращает компактные observations с идентификатором исходного кадра.
- При необходимости геометрии получает точную запись IMU головы.
- Не изменяет локализацию и не выбирает действия робота.

Neural inference остаётся в отдельном `neural-worker`. Его результат содержит
тот же frame identity и объединяется как ещё один источник observations.

### Localization-worker

- Не читает FrameRing и никогда не обрабатывает пиксели.
- Получает детекции, точный head IMU, ориентацию тела и motion odometry.
- Единолично хранит particle filter и координаты на поле.
- Публикует позу робота, covariance/confidence, оценку мяча, видимые landmarks,
  ворота, текущую цель и возраст входов.
- Принимает явный сброс позы и временную целевую точку оператора.
- Не владеет пикселями, камерой, UART и очередями команд тела.

Так исправления локализации можно проверять без риска для камеры и тела. Все
входы являются неизменяемыми записями с source timestamps, а не ссылками на
общие поля `glob`.

### Game-worker

- Владеет ролью и состоянием стратегии, но не аппаратурой.
- Получает snapshots детекции и локализации.
- Отправляет высокоуровневые motion intents в motherboard-worker.
- Может ставиться на паузу, заменяться или перенастраиваться без перезапуска
  камеры, STM, neural inference и локализации.
- Применяет команды оператора только в объявленных safe points, кроме явно
  запрошенной жёсткой остановки.

## Журналирование во время работы

Каждый worker публикует структурированные записи в собственный bounded log
channel. Минимальная запись содержит:

```text
sequence:uint64
monotonic_ns:uint64
level: DEBUG | INFO | WARNING | ERROR | CRITICAL
source: worker/component
message: bounded UTF-8 string
```

Supervisor непрерывно вычитывает channels, добавляет свои записи и сохраняет
короткую bounded history. Stdout/stderr дочерних процессов, включая сообщения
libcamera/GStreamer и traceback, также захватываются supervisor-ом построчно и
помечаются source worker-а. Секреты, session tokens и приватные ключи в логи не
попадают.

Сбор логов для оператора работает всегда. Зеркалирование агрегированного потока
в stdout главного процесса включается отдельным параметром:

```text
logging.stdout_enabled = true
```

При `false` supervisor не печатает runtime log stream в stdout, но продолжает
хранить bounded history и передавать записи подписанному оператору. Вывод
line-buffered и имеет человекочитаемый prefix с monotonic time, level и source.
Log storm не должен блокировать worker или control loop: DEBUG/INFO записи
разрешено отбрасывать с явным dropped counter, а fault/safety events передаются
отдельным высокоприоритетным каналом.

## Путь данных камеры и IMU

Пиксели не передаются через socket и не сериализуются pickle:

```text
                         ┌──> detection-worker
camera-worker ── общий FrameRing ──> neural-worker
                         └──> stream-worker ──> video RTP

detection/neural ── primitives(frame_id) ──> supervisor ──> MessagePack OSD (план)

motherboard-worker ──> общий ImuRing(stm_sequence, IMU data)
  native USB ACM v2 IMU stream; UART используется только загрузчиком STM
  счётчик STM uint16, поле записи shared memory uint32 без продления диапазона

detection-worker ── ObservationRing(frame_id, objects...) ──> localization
localization-worker ── WorldStateRing(...) ──> game + datastream оператора
```

Камера не ждёт IMU перед публикацией кадра и ничего не запрашивает у IMU
provider-а. В первой версии motherboard-worker независимо опрашивает состояние
STM-контейнера, последовательно получает новые IMU records и пишет их в
`ImuRing`. После расширения firmware эту публикацию без изменения контракта
выполняет imu-worker из push-потока второго UART.
Consumer сопоставляет запись с кадром по условию
`uint16(UnicamSequence) == stm_sequence`. Оба счётчика сбрасываются в одной
операции запуска camera pipeline; за длительность матча 16-битный STM counter
не переполняется.

Общий SPMC `FrameRing` имеет 4–8 slot. Camera-worker записывает каждый
`800x650 RGB` кадр один раз, то есть около 90 МиБ/с при 60 кадрах/с. Каждый
consumer имеет собственный cursor/eventfd, копирует новейший стабильный slot в
локальную память перед долгой обработкой и пропускает перезаписанные кадры.
Reference count отсутствует; целостность копии проверяется seqlock. DMA-BUF
zero-copy можно рассмотреть позднее.

Например, при камере 60 FPS и времени detection 50 мс worker может обработать
кадры `100, 103, 106, ...`; кадры между ними не создают очередь. Observation
содержит исходный `UnicamSequence`, поэтому localization выбирает точный IMU
этого кадра из `ImuRing`. Prediction localization по IMU/odometry работает
независимо, а vision correction выполняется только при появлении нового
завершённого observation batch. Устаревший или пришедший не по порядку batch
отбрасывается, а не применяется к текущему состоянию.

`ImuRing` хранит bounded history по 16-битному `stm_sequence` текущего запуска
capture. Активный IMU provider публикует records постоянно, пока runtime camera
pipeline активен; индивидуальных IMU request/response между worker-ами нет.
Детекции, локализация и game state используют bounded latest-only rings. MessagePack по
локальному `SOCK_SEQPACKET` предназначен для команд и дискретных событий, но
не для пикселей.

## Выбранный локальный транспорт

В protocol v1 команды идут через Unix `SOCK_SEQPACKET`, данные — через
shared-memory rings, уведомления — через отдельный `eventfd` каждого
потребителя. POSIX mqueue не используется: его single-consumer семантика,
фиксированные kernel limits и жизненный цикл именованных объектов не решают
задачу истории IMU и latest-only fanout. Полные правила описаны в
[INTERNAL_IPC.md](INTERNAL_IPC.md).

## Перезапуск и сброс camera pipeline

Полный перезапуск камеры не ждёт завершения старых вычислений:

1. Camera-worker немедленно прекращает публикацию кадров и останавливает capture.
2. Supervisor помечает прежний `PipelineContext` неактивным и раздаёт новые
   входные/выходные rings camera, detection, neural и OSD.
3. Worker-ы получают неблокирующий `pipeline.reset`, очищают локальные очереди и
   переключаются на новый context. Старый job знает только старые rings и не
   может записать результат в новую цепочку.
4. Localization очищает ожидающие visual observations, но сохраняет состояние
   particle filter и временно отмечает vision input недоступным.
5. Motherboard-worker останавливает прежний захват, настраивает strobe
   offset/filter, выполняет StartStrobeCapture и StartIMUStream через ACM
   и дожидается подтверждений.
6. Camera-worker запускает новый capture; первые кадры можно отбросить, пока
   consumers переключаются.
7. На начальных кадрах проверяется соответствие sequence Unicam и STM.

```text
stm_sequence = (unicam_sequence + sequence_offset) & 0xffff
frame_id = unicam_sequence в пределах текущего PipelineContext
```

Ожидаемое смещение после reset равно нулю, но его нужно измерить. 16-битный
STM-счётчик при 60 кадрах/с переполняется примерно за 18 минут; bounded history
к этому моменту уже не содержит прежнюю запись с тем же номером.

У каждого worker-а control loop отделён от вычислительного кода и не выполняет
блокирующие detector/GStreamer операции. Если control heartbeat пропал,
supervisor перезапускает процесс. Полный camera restart также закрывает старые
video streams; после запуска создаётся новый `video_stream_id`, поэтому
задержавшийся RTP или OSD старого stream игнорируется без дополнительного поля
эпохи в протоколе.

## Состояния runtime и владение

- `BOOTSTRAP`: reset и запуск аппаратуры.
- `IDLE`: управление и меню работают; камера может быть выключена.
- `MANUAL`: движением владеет оператор; камера включается только по запросу.
- `CALIBRATION`: выполняется управляемая calibration session; связанные
  параметры доступны через key-value API.
- `GAME_READY`: аппаратура готова, роль настроена.
- `GAME_RUNNING`: движением владеет автономная стратегия.
- `GAME_PAUSED`: worker-ы остаются прогретыми, стратегия не выдаёт движения.
- `RECOVERY`: движение остановлено, валидность позы и локализации явная.
- `FAULT`: обязательный worker отказал; supervisor публикует причину.

Для motherboard motion, камеры и neural device существует по одной lease.
Запись параметров сериализует supervisor, а изменять их может только владелец
управляющей lease. Переход режима передаёт lease, не перезапуская несвязанные
worker-ы.

## GIL и планирование

Free-threaded Python не требуется. У каждого worker собственный интерпретатор и
GIL. Supervisor использует `asyncio` для sockets, timers и worker events, а
CPU-тяжёлые компоненты работают независимо на обычном CPython.

Ни одна внутренняя очередь не может расти без ограничения. Камера и телеметрия
latest-only; изменяющие состояние команды упорядочены и дедуплицированы;
длительные тесты представлены jobs с progress и cancellation.
