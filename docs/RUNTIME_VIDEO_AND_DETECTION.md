# Runtime-видео и первый детектор

Состояние 26.09.2026. Код Python в roki-ng, отгружен на голову 172.30.0.1.
Тело отключено; ни один из этих тестов не отправляет движения.
**RTP OSD не реализовывать**: пользователь будет менять его транспорт.

Команды в примерах обновлены 01.10.2026. Текущий полный контракт:
[CAMERA_VIDEOSTREAM_PROTOCOL.md](CAMERA_VIDEOSTREAM_PROTOCOL.md).
Замеры ниже относятся к версии на дату измерения, не являются проверкой
нового многопользовательского API на железе.

## Что работает

Пять процессов: supervisor, motherboard, camera, stream, detection. При старте
программы камера, кодер и детектор выключены. Команды доступны через общий
UDP MessagePack endpoint; данные камеры идут через iceoryx2, не supervisor.

Camera-worker владеет libcamera и публикует полный FOV 1600x1300 RAW10,
уменьшенный ISP до 800x650 BGR. Его запуск/синхронизация IMU описаны отдельно
в [CAMERA_IMU_CAPTURE.md](CAMERA_IMU_CAPTURE.md).

Пути видео, проверенные в этом отчёте:

- direct-gst: libcamerasrc самостоятельно захватывает видео; IMU не нужен.
- runtime: читает уже запущенный camera-worker через shared memory. Pipeline:
  appsrc BGR -> bounded queue -> v4l2convert -> I420 -> v4l2jpegenc/v4l2h264enc
  -> RTP payloader -> multiudpsink. Parse-элементов на отправителе нет.

Также теперь доступен source=localisation, см. [LOCALISATION_VIDEO.md](LOCALISATION_VIDEO.md).

Runtime videostream.start не меняет сенсор, exposure/gain или захват IMU.
Videostream.stop закрывает только encoder/subscriber. Camera.stop сначала закрывает
runtime-видео и детектор, затем libcamera и IMU capture. Повторный запуск видео
не требует заново привязывать Unicam/STM.

GStreamer/appsrc и iceoryx2 имеют разные владельцы памяти. Здесь нет обещания
zero-copy до аппаратного кодера: в stream-worker выполняется копирование в
Gst.Buffer. Передача через bytes использует быстрый bulk-copy PyGObject;
memoryview в Buffer.fill оказался очень медленным: около 169 мс против
2,7 мс на 1,56 МБ в коротком микротесте на CM4. Этот путь исправлен.

Читатели stream/detection используют FrameReader: отдельный поток с iceoryx2
WaitSet и stop pipe. Они не опрашивают shared memory таймером. Медленный читатель
берёт последний доступный кадр, освобождая старые; камера его не ждёт.
Очереди appsrc/GStreamer ограничены и пропускают старые данные вместо накопления
секунд задержки. Остановка явно будит WaitSet и дожидается завершения читателя.

## Детектор

Detection-worker выполняет BGR -> LAB -> inRange -> connected components.
L-пороги совпадают по шкале со старым кодом: 0..100 с переводом в OpenCV 0..255;
a/b -128..127 переводятся прибавлением 128. Это не HSV и не RGB-пороги.

Профили: orange_ball, green_field, white_marking, blue_posts, yellow_posts,
white_posts. Профиль выбирается при detection.start. Максимум четыре области
в результате, плюс полное число прошедших фильтр. Проверяются и foreground
pixels_min, и площадь bounding box. В старом reload.Image.find_blobs параметр
area_threshold фактически игнорировался; здесь он действительно применяется.

Это **кандидаты по цвету**, не законченный футбольный детектор. Не перенесены
проверка зелёного поля около мяча, выбор единственного мяча, проекция через
калибровку/IMU, геометрия ворот, линии и нейродетектор. До их реализации нельзя
выдавать цветовой blob за проверенную позицию мяча в мировых координатах.

Детектор сейчас не требует IMU: результат выражен только в пикселях и содержит
Unicam sequence. Будущая локализация должна связать его с IMU точного кадра.
Сами изображения не рисуются и не изменяются. Ни RTP OSD, ни overlay в кадре нет.

Результаты публикуются в roki/detection/blobs/v1 через iceoryx2 и доступны
оператору через detection.status/data.subscribe(detection.state). Журнал остаётся
общим. При остановке результат очищается. OpenCV загружается только при старте
детектора, использует один вычислительный поток внутри отдельного процесса.

## Команды оператора

После control.acquire и mode.set(MANUAL):

```text
camera.start {}
detection.list {}
detection.start {"profile":"orange_ball"}
params.keys {"prefix":"vision.orange_ball."}
params.set {"key":"vision.orange_ball.pixels_min","value":100}
data.subscribe {"topic":"detection.state","rate_hz":2}
videostream.create {"source":"runtime","codec":{"name":"jpeg"},"output":{"width":800,"height":648,"fps":30}}
videostream.start {"stream_id":"ID ИЗ ОТВЕТА","rtp_port":5004}
videostream.stop {"stream_id":"ID ИЗ ОТВЕТА"}
detection.stop {}
camera.stop {}
```

Reference Client добавляет lease_epoch. Начальные пороги в коде не взяты из
калибровки конкретного робота: их нужно настроить на поле. Пользовательские
параметры сохраняются только в state-dir, не в репозитории.

Runtime max_fps задаёт верхнюю частоту видео: лишние кадры пропускаются до
копирования. Его можно менять через videostream.update без перезапуска, в
пределах output.fps. На start нельзя запросить больше частоты работающей камеры.
Выход не больше 800x650. Для JPEG
размер должен быть кратен 8, поэтому проверен 800x648; H.264 проверен 800x650.
GStreamer может задавать pixel-aspect-ratio при масштабировании; оператору
следует учитывать negotiated caps. Сенсор при этом не переключается и FOV не
обрезается до 1280x720.

## Проверки

На голове для диагностического запуска (не параллельно с другим runtime):

```sh
cd /opt/roki-ng
python3 -m roki_ng --body-disabled --skip-mixing --state-dir /tmp/roki-ng-capture-check
```

На ПК:

```sh
cd roki-ng
python tools/check_runtime_video.py --robot 172.30.0.1 --codec jpeg --detection
python tools/check_runtime_video.py --robot 172.30.0.1 --codec h264 --detection
```

Это настоящий UDP control и приём RTP на ноутбуке. Каждый тест декодирует два
последовательных запуска видео, проверяет размеры, сохранение camera/IMU после
video.stop и остановку video/detection после camera.stop. `--frames 1800`
увеличивает каждый прогон примерно до минуты при 30 FPS.

JPEG/H.264 прошли совместно с детектором; отдельный consumer на голове прочитал
MessagePack результатов из iceoryx2. В снимках состояния LAB-обработка занимала
примерно 14..18 мс после прогрева; первый вызов был существенно медленнее.
Это отдельные измерения, не гарантия производительности для всех изображений.
Сцена не подготовлена для проверки мяча: качество детекции на поле не проверялось.

Длинный совместный прогон: H.264 800x650, два окна по 1800 декодированных кадров
на ноутбуке, **30,0 FPS в каждом**. За оба окна детектор обработал 6760 кадров;
камера работала около 60 FPS, отстающий детектор пропускал кадры штатно.
IMU сохраняла synced, video.stop/start между окнами не перезапускал камеру.

На ПК дополнительный тест синтетического красного BGR кадра проходит полный
appsrc -> JPEG/RTP -> JPEG decode и проверяет каналы цвета. Юнит-тесты проверяют
пороговые диапазоны, площади, точный ID кадра, очистку результата и откат параметров
при ошибке записи. Они не заменяют полевую калибровку камеры.

Финальный локальный набор: 68 pytest-тестов проходят; compileall проходит.
Остаётся предупреждение GI о deprecated GLib.unix_signal_add_full.
Код отгружен в /opt/roki-ng. Autostart/dinit не менялся, тестовые процессы
завершены. После финальной проверки capture_active=false, stream_mode=0,
body_queue_size=0, imu_healthy=true. Полные логи скопированы в
out/motherboard-acm-v2-20260926 общего workspace.

## Что дальше

1. Перенести проверку мяча на поле и другие семантические детекторы из исходника,
   отделив пиксельные наблюдения от проекции в реальные координаты.
2. Добавить calibration model и localization-worker с точным IMU join, затем
   game-worker. В текущем детекторе не использовать «последнюю IMU» вместо точной.
3. Подключить GUI настройки LAB и requested datastream к уже описанному API.
   OSD-транспорт пока не выбирать и не реализовывать.
4. С телом проверить очередь, движения и реконнект; на этом этапе тела нет.

После SIGKILL motherboard firmware всё ещё не узнаёт о смерти владельца USB.
Камера останавливается supervisor-ом, а следующая инициализация явно делает
StopStrobeCapture. Watchdog владельца на STM пока не реализован; см. отчёт
проверки отказов в CAMERA_IMU_CAPTURE.md.
