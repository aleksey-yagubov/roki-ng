# roki-ng

Ближайшие работы с телом: [очерёдность этапов](docs/BODY_CONTROL_ROADMAP.md).
Первый этап: [экспериментальное удержание приседа по IMU](docs/CROUCH_STABILIZATION.md)
(выключено по умолчанию, требуется физическая проверка).

Новое поколение runtime-системы робота Roki-2 и приложения оператора.

Этот репозиторий представляет собой чистое перепроектирование, а не изменение
`Roki_2_Soccer` на месте. Старый репозиторий остаётся эталоном поведения и
источником алгоритмов, которые будут переноситься по одному вместе с тестами.

Реализован Python runtime: supervisor, motherboard-worker, camera-worker,
stream-worker и detection-worker.
Он поддерживает ручные движения и тесты, UDP/MessagePack управление, запрашиваемое
RTP-видео через `libcamerasrc`, параметры, jobs, логи и базовые datastream.
Также работает видео из runtime-кадров и базовый LAB-детектор цветовых областей.
Runtime-захват libcamera и IMU передаются через iceoryx2, смещение счётчиков
определяется при каждом запуске камеры. Qt-приложение разрабатывается отдельно.
Старые сервер/клиент с этим протоколом несовместимы.

Точный API для нового клиента: [OPERATOR_PROTOCOL.md](docs/OPERATOR_PROTOCOL.md).
Камера и передачи: [CAMERA_VIDEOSTREAM_PROTOCOL.md](docs/CAMERA_VIDEOSTREAM_PROTOCOL.md).
Для GUI-агента: [GUI_CAMERA_VIDEOSTREAM_HANDOFF.md](docs/GUI_CAMERA_VIDEOSTREAM_HANDOFF.md).
Архитектура следующих этапов: [`docs`](docs/README.md).

## Проверка на ПК

Нужны Python >=3.11 и `msgpack`. Устанавливать проект необязательно:

```sh
git clone git@github.com:aleksey-yagubov/roki-ng.git
cd roki-ng
python -m roki_ng --simulate --host 127.0.0.1 --state-dir /tmp/roki-ng-demo
```

В другом терминале из того же каталога:

```sh
python -m roki_ng.client --robot 127.0.0.1 --control
```

Примеры строк в интерактивном клиенте:

```text
params.set {"key":"logging.stdout_enabled","value":true}
data.subscribe {"topic":"motion.state","rate_hz":2}
motion.pose {"name":"base_stand"}
motion.slots {}
test.list {}
test.describe {"name":"run_test"}
test.start {"name":"run_test","mode":"spot"}
motion.stop_graceful {}
motion.stop_hard {}
```

Client автоматически добавляет lease_epoch, поддерживает heartbeat и выводит
события. `quit` закрывает сессию. Класс `Client` также предоставляет `drive()`
для будущей обработки клавиш в Qt; samples следует посылать примерно 20 раз/с.

Для настоящего локального GStreamer-теста добавить supervisor флаг `--test-video`.
Он использует videotestsrc и программный encoder, не открывает камеру. Пример
подписки на прямой выход stream-worker:

```text
videostream.subscribe {"name":"stream","rtp_port":5004,"settings":{"fps":30}}
```

Для этого теста нужны GI/GStreamer, x264enc/rtph264pay/multiudpsink и
videoconvert. Обычный `--simulate` не требует GI, Roki или starkit и не передаёт
настоящего видео.

## Запуск на роботе

Нужны установленные `msgpack`, `Roki`, `starkit.alpha_calculation`, pyserial,
libgpiod Python v2, GI/GStreamer и плагины из Buildroot. Для camera-worker нужны
libcamera с UnicamSequence, numpy и iceoryx2. Требуется новая Roki с ACM v2.
Для detection-worker дополнительно нужен OpenCV; он не импортируется до запуска детектора.
Ничего компилировать
для самого runtime не требуется. При работе из каталога проекта:

```sh
python -m roki_ng --uart /dev/ttyAMA5 --state-dir /var/lib/roki-ng
```

Сначала supervisor находит линию `userspace-motherboard-reset` по имени из
Device Tree, выполняет reset и CMD:START_FW на 921600. Номер линии и gpiochip
не зафиксированы; `--gpiochip` опционально ограничивает поиск одним устройством.
При отсутствии или неоднозначности имени запуск завершается ошибкой, без
подстановки числового GPIO. Линия `userspace-bluecoin-reset` не запрашивается
и не изменяется: BlueCoin остаётся в reset, заданном загрузчиком.
Затем motherboard-worker открывает USB ACM, найденный нативной библиотекой по
product `Roki motherboard`, и сохраняет соединение до выхода. UART нужен только
загрузчику. `--skip-bootstrap` допустим, если firmware уже запущена. Другой runtime
не должен одновременно владеть motherboard/камерой. BlueCoin не нужен.

Для проверки прямых команд серв без запуска служебного слота контроллера есть
`--skip-mixing`. Он не запускает mixing, но не останавливает уже работающий слот.
Это диагностический режим, без гарантии стабилизации, которую обеспечивает mixing.

Видеопротокол использует именованные выходы воркеров и только H.264.
Каталог videostream.list запрашивается у производителей, включая недоступные
выходы. Один encoder на выход обслуживает всех получателей через multiudpsink.

Прямой выход stream-worker захватывает камеру через libcamerasrc без IMU.
По умолчанию: полный sensor 1600x1300 RAW10, обработанное видео 800x650@60,
H.264 2 Mbit/s. Настройки захвата доступны до запуска; аппаратную поддержку
окончательно проверяет GStreamer. Рабочая камера и прямой захват взаимоисключаются.

Отдельный camera.start запускает **1600x1300 RAW10 -> 800x650 BGR** в shared memory,
обязательно с IMU. Пиксели доступны сразу, точное сопоставление разрешено после
camera.synchronized. См. [CAMERA_IMU_CAPTURE.md](docs/CAMERA_IMU_CAPTURE.md).

После camera.start: videostream.subscribe с name="camera" и rtp_port.
Это передача рабочих кадров без повторного захвата и без изменения размеров.
Detection.start включает LAB connected components; параметры vision.* применяются
со следующего кадра. Это цветовые кандидаты, не классификация мяча.

Владелец управления запускает, настраивает и останавливает выходы; наблюдатель
подписывается на работающий выход. Последний unsubscribe/истечение сессии
останавливает кодер и запрошенный preview, но не рабочую камеру и алгоритмы.
Max_fps меняется без рестарта; fps и bitrate задаются до запуска.
Каталог содержит controls с признаками fixed/live для GUI.
Контракт: [CAMERA_VIDEOSTREAM_PROTOCOL.md](docs/CAMERA_VIDEOSTREAM_PROTOCOL.md).
Передача GUI-агенту: [GUI_CAMERA_VIDEOSTREAM_HANDOFF.md](docs/GUI_CAMERA_VIDEOSTREAM_HANDOFF.md).

OSD согласован через MessagePack; в RTP планируется только UnicamSequence
для привязки. Точное OSD и эта RTP extension ещё не реализованы.

Параметры конкретного робота автоматически создаются вне репозитория в state-dir.
Заводские начальные значения не заменяют его калибровку. Структурированные логи
доступны клиенту всегда, зеркало stdout включается ключом logging.stdout_enabled.

## Границы первого этапа

- Математика ходьбы/обычного удара и 57 программных slots перенесены отдельно;
  старое приложение не импортируется. Происхождение: [IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md).
- Head, прыжки, crouch/stand/base_stand и ограниченные циклические тесты доступны
  без камеры. Движения не накапливаются в очереди кликов.
- Старые тесты сгруппированы в run_test, jump_test, rotation_test и kick_test.
  Полный каталог и параметры выдаёт робот. Калибровка поворотов сохраняется
  supervisor атомарно, ручные замеры принимаются через test.measure.
  Режим new_kick пока недоступен: нужны детектор мяча и аппаратный слот 31.
- Hard stop прекращает выдачу поз и очищает STM queue; не прерывает уже принятую
  сервой интерполяцию или аппаратный slot контроллера. Mixing запускается один раз.
- Физическое голосовое меню кнопок уже работает, включая выбор игры и тестов;
  реализация автономного футбола ещё впереди. Есть отдельный get_up_test.
  Автоматическая игровая логика падения/вставания, параметры серв и OSD пока
  не реализованы. Qt GUI ведётся отдельно.
- Тесты на ПК не подтверждают механическую устойчивость gait и работу оборудования.
  Runtime camera/STM проверены на голове без тела; обмен и небольшие движения
  проверялись на стенде. Полноценная проверка ходьбы на исправном теле ещё нужна.

## Совместная разработка

Границы алгоритмов и записка агенту GUI:
[COLLABORATION_HANDOFF.md](docs/COLLABORATION_HANDOFF.md).
Задание сборке образа: [BUILDROOT_HANDOFF_2026_09_27.md](docs/BUILDROOT_HANDOFF_2026_09_27.md).
Рабочие параметры хранятся в `/var/lib/roki-ng/`, в Git поставляются defaults.
Калибровки оптики планируются как готовые `.npy` и JSON-метаданные, не пересчёт
при каждом старте; профили поля редактируются отдельно от алгоритма локализации.

Протокол рассчитан на доверенную сеть робота: session token не заменяет
криптографическую аутентификацию. Не открывайте UDP управления в Интернет.
Происхождение перенесённых алгоритмов указано в IMPLEMENTATION_STATUS.md;
публикация fork не назначает новую лицензию чужому коду.

## Тесты

```sh
python -m pytest -q
```

Тесты запускают настоящие дочерние процессы и UDP sockets, проверяют дедупликацию,
lease, отмену, deadman, очистку ресурсов, параметры и генераторы gait. При наличии
GStreamer с x264enc также проверяется реальная RTP/H.264-передача через loopback, SSRC, MTU,
DF и восстановление pipeline после ошибки.

Состав, зависимости и границы этих проверок: [TESTING.md](docs/TESTING.md).
