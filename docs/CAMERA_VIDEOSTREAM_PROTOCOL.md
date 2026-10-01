# Камера и видеопередача

Реализованный контракт от 01.10.2026. Заменяет прежние `video.*` без алиасов.
Оболочка MessagePack, heartbeat, lease, пагинация и ошибки описаны в
[OPERATOR_PROTOCOL_V1.md](OPERATOR_PROTOCOL_V1.md). Здесь приведены тела запросов;
`lease_epoch` обязателен для операций управления. Размер всей UDP-датаграммы
MessagePack не превышает 1400 байт. Списки нужно читать до `next_offset=null`.

## Разделение ответственности

- `camera.*`: runtime-захват libcamera, синхронная IMU и настройки ISP.
- `videostream.*`: каталог источников, определения передач, кодирование и RTP.
- Просмотр: только окно GUI. Несколько окон используют один приёмник/декодер
  одного стрима. Создание и закрытие окна не вызывает запросов роботу.

`hello`, каталоги, status, создание определения стрима и подписка на datastream
не запускают захват, детектор, локализацию или передачу видео.

## Права и время жизни

Только текущий владелец управления вызывает `camera.start/stop`, изменения ISP,
`videostream.create/start/update/stop/destroy`. У стримов нет отдельного владельца:
после передачи управления новый владелец может управлять любым определением.
Состояния, каталоги и `videostream.attach/detach` доступны всем сессиям.

`start` запускает передачу и добавляет первого получателя: текущую сессию и
заданный ею UDP-порт. `attach` добавляет наблюдателя к уже запущенной передаче,
но не запускает остановленную. `detach` удаляет только текущую сессию.
Одна сессия имеет один адрес назначения на стрим. Для изменения порта сначала
выполнить detach. IP всегда берётся из адреса управляющей сессии; произвольный
destination/host запрещён. Разные передачи не могут слать на один IP:порт.

Потеря lease сама по себе не удаляет получателей. При `session.close` или
истечении heartbeat удаляется получатель этой сессии. Если остались другие,
передача продолжается с теми же run_id, SSRC и кодировщиком. Последний получатель
отключился: pipeline останавливается, определение остаётся в списке stopped.
Для direct-gst при этом освобождается камера. Для runtime камера/IMU продолжают
работать. Для локализации прекращается только рисование/публикация видео, если
другие передачи его не используют; вычисление локализации продолжается.

Остановка владельцем управления отключает всех получателей выбранной передачи.
После нового start наблюдатели должны явно подключиться заново. Автономные
вычисления не останавливаются из-за отключения видео или оператора.

## Камера

Геометрия фиксирована: сенсор **1600x1300 RAW10**, ISP **800x650 BGR**.
`camera.start` всегда включает захват и привязку IMU. Параметра `with_imu` нет;
для ручного видео без IMU используется источник direct-gst.

| Команда | Тело | Результат |
| --- | --- | --- |
| camera.capabilities | {} | sensor, output, geometry_mutable=false, imu_required=true, frame_duration_us, controls_probed |
| camera.start | lease_epoch, необязательные frame_duration_us/exposure_us/gain | Состояние запущенного захвата; нужны MANUAL и свободная камера |
| camera.stop | lease_epoch | Остановка захвата, IMU, зависимых детекции/локализации и их передач |
| camera.status | {} | Состояние camera-worker, не stream-worker |
| camera.controls.list | offset=0, limit=2 | items, total, next_offset; максимум 2 controls на страницу |
| camera.controls.set | lease_epoch, values | Применить временные controls; values, saved=false |
| camera.controls.save | lease_epoch, values | Применить и атомарно сохранить указанные controls; values, saved=true |
| camera.controls.freeze | lease_epoch, group=all/exposure/white_balance | Зафиксировать свежие измеренные AE/AWB в ручные controls; values, saved=false, source_sequence |

Пример запуска: `{"lease_epoch":1,"frame_duration_us":16667}`.
Период 8333..100000 мкс, default 16667; эти программные границы не обещают
поддержку всех частот сенсором. На железе ранее проверены 16667/33333 мкс.
Период меняется только через stop/start. Повтор совместимого start не
перезапускает камеру; изменение явных стартовых настроек даёт restart_required.
Нельзя одновременно захватывать камеру из camera-worker и direct-gst:
конфликт не останавливает текущего владельца автоматически.

`camera.status`: running, prepared, frames, bad_frames, sequence,
frame_duration_us, error, topic, imu_sync, requested_controls,
measured_controls, measured_age_ms. Datastream `camera.state` содержит это
состояние. `imu_sync.state`: disabled при остановке, затем aligning, matched,
synced. Пиксели доступны до synced, точное сопоставление IMU пока запрещено.
Подробности: [CAMERA_IMU_CAPTURE.md](CAMERA_IMU_CAPTURE.md).

Доступные ключи controls:

| Ключ | Программный диапазон | Default |
| --- | --- | --- |
| camera.exposure_us | 1..100000, не больше периода кадра | 8000 |
| camera.analogue_gain | 1..16 | 1 |
| camera.ae_enabled | bool | false |
| camera.awb_enabled | bool | false |
| camera.white_balance.red_gain | 0.01..32 | 1 |
| camera.white_balance.blue_gain | 0.01..32 | 1 |

`controls.list.items[]`: key, type, min, max, default, value, supported,
apply="next_request", description. Value означает запрошенное, не измеренное.
До открытия камеры supported=null и границы только программные. После открытия
они пересекаются с libcamera ranges; supported=false означает отсутствующий
control. Запрос каталога не открывает камеру ради probe.

Пример: `camera.controls.set {"lease_epoch":1,"values":{"camera.ae_enabled":false,"camera.exposure_us":7000}}`.
Подтверждение означает постановку controls для следующих свободных Requests,
а не немедленное изменение уже обрабатываемого ISP кадра. При включённой
автоматике ручные значения не описывают её текущий результат.

В measured_controls: sequence, sensor_timestamp_ns, exposure_us, gain,
colour_gains; отсутствующая метадата равна null. Freeze требует работающей
камеры и метаданных не старше 1000 мс. Для сохранения после freeze передать
возвращённые values в controls.save. Временные значения не пишутся в JSON;
перезапуск worker получает сохранённые. Controls не относятся к direct-gst.
Явный params.set этих же ключей по-прежнему применяет и сохраняет значения.

## Источники

`videostream.sources {"offset":0,"limit":1}` возвращает каталог возможностей,
не список передач. Максимум один элемент на страницу. Известные неработающие
источники тоже присутствуют, с available=false и reason.

| id | Кто публикует | Условие запуска передачи |
| --- | --- | --- |
| direct-gst | stream-worker/libcamerasrc | Камера свободна; capture создаётся при start |
| runtime | camera-worker | Уже запущен camera.start |
| localisation | localisation-worker | Уже работают камера и localisation.start |

Пример элемента (сокращён):

```json
{"id":"localisation","worker":"localisation","name":"Разметка локализации",
 "topic":"roki/localisation/frame/v1","parameters":{},
 "requires":["camera.running","localisation.running"],
 "available":false,"reason":"localisation_not_ready",
 "output":{"requested":false,"publishing":false,"frames":0,"age_ms":null,"error":null},
 "stream_settings":{"codecs":["h264","jpeg"],"max_fps":[1,120],
 "max_size":[800,650],"jpeg_alignment":8,"live_update":["max_fps"]}}
```

Reason: worker_unavailable, camera_state_unknown, camera_busy, camera_stopped,
localisation_not_ready, source_fault либо null. Available означает возможность
запустить новую передачу сейчас, не состояние уже работающей. Например,
direct-gst при активном захвате недоступен для второй передачи, но к первой
можно attach. Hardware encoder ещё может отказать при фактическом запуске.

`output` описывает производителя, а `videostream.status` описывает encoder/RTP.
Для runtime output содержит publishing/frames; для direct-gst publishing.
Для localisation requested не равно publishing: запрос получен, но первого
обработанного кадра ещё может не быть. Age_ms показывает возраст публикации.
Publishing требует запрошенного выхода и кадра за последние 2 секунды.

Внутренний IPC использует `source.list` на фиксированных сокетах воркеров;
статический реестр сохраняет отсутствующие источники в каталоге. Для
localisation supervisor вызывает source.start/stop/status. Stream-worker
подписывается на shared memory. Рисование требует одновременно явного запроса
и живого подписчика iceoryx2; без спроса loans/копирование не выполняются.
Последний зависимый pipeline остановился: source.stop, не localisation.stop.

Параметры видеовыходов пока пусты (`parameters:{}`). Детектор сейчас выдаёт
данные, своего обработанного изображения не имеет. Маски, remap fisheye и
эталонный поток для threshold tuner пока не реализованы и не рекламируются.

## Передачи

```text
videostream.create {"lease_epoch":1,"source":"runtime","output":{"width":800,"height":650,"fps":60},"max_fps":15,"codec":{"name":"h264","bitrate":2000000},"mtu":1400}
videostream.start {"lease_epoch":1,"stream_id":"...","rtp_port":5004}
videostream.attach {"stream_id":"...","rtp_port":5004}
videostream.update {"lease_epoch":1,"stream_id":"...","max_fps":5}
videostream.detach {"stream_id":"..."}
```

Defaults create: source=direct-gst; sensor width1600/height1300/depth10;
output width800/height650/fps60; codec h264/bitrate2000000; mtu1400;
parameters={}; max_fps=output.fps. Sensor передаётся **только** direct-gst.
Его допустимые размеры: width320..4096, height240..4096, depth8/10.
Output чётный: width160..1600, height120..1300, не больше sensor;
runtime/localisation дополнительно не больше 800x650. FPS1..120.
H.264 bitrate100000..20000000 bit/s; profile high, level4.2.
JPEG bitrate не поддерживается; размеры RTP/JPEG кратны 8, например 800x648.
MTU576..1400 включает RTP header, не включает UDP/IP. На IPv4 установлен DF.

| Команда | Аргументы помимо lease | Результат |
| --- | --- | --- |
| videostream.capabilities | {} | sources, codecs, defaults, max_streams=8, max_receivers=4, live_update, exact_osd=false, rtcp=false |
| videostream.list | offset=0, limit=4 | items, total, next_offset; максимум 4 кратких описания |
| videostream.create | source, sensor/output/codec, max_fps, parameters, mtu | Определение, state=created, run_id/ssrc=null; без получателя |
| videostream.start | stream_id, rtp_port | Полный status; normally starting, первый получатель добавлен |
| videostream.attach | stream_id, rtp_port | Status; только начатая передача, включая starting |
| videostream.detach | stream_id | Status после удаления текущего получателя |
| videostream.status | stream_id | Полный status |
| videostream.update | stream_id, max_fps и/или bitrate | Status; ограничения ниже |
| videostream.stop | stream_id | stream_id, state=stopped; определение сохраняется |
| videostream.destroy | stream_id | stream_id, state=destroyed; определение удалено |

Элемент list: stream_id, source, state, run_id, receivers, attached.
Полный status: stream_id, state, spec (нормализованный create), run_id, ssrc,
payload_type (H264=96, JPEG=26), encoding_name, clock_rate=90000, exact_osd=false,
error; дополнительно packets, receivers, attached, destination текущей сессии
([IP,port] или null), actual_fps, frames_submitted, frames_skipped. После первого
RTP появляется negotiated_caps входа кодировщика. Spec содержит запрошенные
настройки; согласованные caps и measured metadata нельзя заменять ими.

Состояния: created, starting, running, stopped, failed. На каждом новом start
генерируются новые run_id и SSRC; stream_id живёт до destroy/рестарта worker.
Повтор start активного стрима только подключает вызывающую сессию.
Повтор attach того же адреса идемпотентен. Новый порт без detach даёт conflict.

`max_fps` применяется к runtime/localisation **до копирования BGR и кодирования**.
Читатель получает новые descriptor-ы, но лишние кадры не копирует в GstBuffer.
Обновление не сбрасывает pipeline, PTS, run_id или SSRC. Диапазон от 1 до
output.fps; output.fps является фиксированным номинальным потолком caps кодера.
Реальная частота может быть ниже из-за источника/нагрузки. На start max_fps не
должен превышать частоту runtime-камеры. Повторов кадров нет. Direct-gst не
принимает max_fps: его output.fps задаёт capture caps и меняется пересозданием.
Bitrate H.264 меняется update только после stop (иначе restart_required).
Codec, геометрия и источник меняются через destroy/create.

Одна передача = один encoder/payloader и multiudpsink на всех получателей.
Одинаковые адреса двух сессий не удваивают пакеты. Разные передачи одного или
разных runtime-источников разрешены; нет программного лимита в один H.264.
Лимит 8 определений не гарантирует 8 работающих аппаратных кодировщиков:
ограничения памяти, shared memory подписчиков и железа обнаруживаются при start.

`actual_fps` измеряется на выходе encoder, не у получателя. Packets считает RTP
до размножения адресатам, не подтверждает доставку. События videostream.started
содержат stream_id/run_id, stopped содержит stream_id/reason, failed содержит
stream_id/error. Для надёжного UI после события запросить status/list:
события по UDP могут потеряться. Datastream `videostream.state` содержит
active_streams [{stream_id,source,state}], суммарные packets/frames_submitted/
frames_skipped. Ошибка pipeline закрывает его, но не процесс stream-worker.
Отсутствие RTP более 10 секунд также завершает pipeline с failed.

Ошибки: not_found (нет определения), not_ready (нет источника/активной передачи),
busy/camera_busy (занят захват или достигнут лимит), invalid_argument,
not_supported, restart_required, conflict, source_fault, pipeline_error,
а также стандартные ошибки сессии/lease/worker. После failed возможен повтор
start; после worker restart старые stream_id недействительны.

## Границы этого этапа

UnicamSequence в RTP и новый MessagePack OSD не реализованы этим изменением:
exact_osd=false. Видео локализации пока содержит рисование внутри кадра.
RTCP, управление JPEG quality/image_encode и lossless threshold transport
также не добавлялись. GUI не должен выдавать этот поток за цветовой эталон.
multiudpsink входит в тот же установленный GStreamer UDP plugin, что udpsink;
отдельного нового Buildroot-пакета для этого изменения не требуется.
