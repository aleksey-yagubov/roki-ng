# Камера и именованные видеовыходы

Текущий контракт, 02.10.2026. GUI и roki-ng обновляются синхронно.
Версий API и согласования совместимости нет. Старые create/start/attach/detach/destroy/sources
удалены, алиасов нет. Каталог и подписки не являются сетевым discovery.
Полный UDP-пакет ограничен 1400 байт, установлен DF. Тела запросов ниже
нужно помещать в обычный конверт [OPERATOR_PROTOCOL.md](OPERATOR_PROTOCOL.md).

## Модель и права

Один воркер объявляет не больше одного видеовыхода через IPC `source.list`.
Имя выхода равно имени воркера в конфигурации supervisor. Supervisor опрашивает
известных peers, не хранит справочник возможных выходов. Нет произвольных
пользовательских объектов с UUID, create/destroy и отдельных настроек получателей.
Если воркер ещё ни разу не ответил, его неизвестный выход не выдумывается.
Последнее полученное описание сохраняется в памяти supervisor: при недоступности
воркера выход остаётся в каталоге с `available=false`. Перезапуск supervisor
очищает этот кеш; затем описания снова запрашиваются.

Сейчас свои выходы объявляют stream-worker (прямой libcamerasrc без IMU),
camera-worker (рабочие кадры) и localisation-worker (диагностическое видео).
Это **примеры текущих производителей, не перечисление допустимых имён в API**.
Detection пока не объявляет видео. Будущий производитель добавляет своё описание,
без расширения списка имён в supervisor или GUI.

Кодирование всех выходов остаётся в stream-worker. Один H.264-кодер и RTP-payloader
на каждый активный выход; все получатели делят его через multiudpsink.
Разные выходы можно смотреть одновременно, если хватает аппаратных ресурсов.
Конфликт захвата камеры не вытесняет её текущего владельца.

Наблюдателю доступны каталог, status, подписка на уже работающий выход и
отписка от собственного приёма. Запуск остановленного выхода, update и stop
для всех требуют актуальную управляющую lease. Потеря lease сама по себе
не прерывает просмотр. IP получателя берётся из сессии, не из тела запроса.

## Команды

| Команда | Тело (кроме lease_epoch) | Результат |
| --- | --- | --- |
| videostream.capabilities | {} | codec=h264, mtu=1400, max_receivers=4, exact_osd=false, rtcp=false |
| videostream.list | offset=0, limit=1 | items, total, next_offset; максимум 1 выход на страницу |
| videostream.status | name | Состояние именованного выхода и RTP |
| videostream.subscribe | name, rtp_port, необязательный settings | Запуск/подключение текущей сессии; полный status |
| videostream.unsubscribe | name | Отписка текущей сессии; полный status |
| videostream.update | name, settings | Изменение общих настроек владельцем; полный status |
| videostream.stop | name | Остановка для всех и удаление получателей; полный status |

Для операций управления нужен `lease_epoch`. RTP-порт: integer 1024..65535.
Незнакомые поля запроса/настроек отклоняются. `codec`, `source`, `stream_id`,
`destination`, `host`, `output` не являются полями нового API.
Кодек только **H.264**, JPEG удалён. MTU фиксирован 1400.
Не более четырёх сессий-получателей на выход.

### Каталог и controls

Каждый items[] содержит:
`name, title, state, available, reason, settings, controls, receivers, subscribed, producer`.

`settings` — текущие общие настройки; до первого запуска это дефолты
производителя. Последние настройки сохраняются после остановки, но не после
перезапуска stream-worker. Настройки видео не записываются в JSON автоматически.
`controls` — описание разрешённых полей, которое предоставляет производитель.
Не делать в GUI собственный список выходов, геометрии или разрешённых настроек.

Пример части каталога camera-worker:

```json
{
  "name": "camera",
  "title": "Камера",
  "state": "stopped",
  "available": false,
  "reason": "camera_stopped",
  "settings": {"width":800,"height":650,"fps":60,"max_fps":60,"bitrate":2000000},
  "controls": {
    "width": {"type":"int","fixed":true},
    "height": {"type":"int","fixed":true},
    "fps": {"type":"float","min":1,"max":120,"live":false},
    "max_fps": {"type":"float","min":1,"max":120,"live":true},
    "bitrate": {"type":"int","min":100000,"max":20000000,"live":false}
  },
  "receivers":0,
  "subscribed":false,
  "producer":{"publishing":false,"requested":false}
}
```

Правила GUI:
- Значение брать из settings; тип и диапазон — из controls. Возможен массив choices.
- `fixed=true`: поле только для чтения даже до запуска.
- `live=true`: владелец может менять поле во время starting/running.
- Иначе поле редактируемо владельцем только при остановленной передаче.
- Наблюдателю все поля read-only. Работающий выход всегда показывает его
  общие текущие settings, а не оставшийся локальный черновик.
- Разделять controls на разные панели не требуется.

Camera и localisation задают фиксированные **800x650**. Resize и подмена на
800x648 запрещены; никакого padding нет. Другой producer может объявить другую
фиксированную геометрию. Прямой захват сейчас допускает width/height/fps и
sensor_width/sensor_height/sensor_depth; эти поля не появляются у рабочих кадров.
Границы в controls программные, успешный аппаратный запуск проверяет GStreamer.

У shared-frame выходов `fps` — потолок caps кодировщика при запуске,
`max_fps` — изменяемый предел подачи кадров: 1..fps, также не выше частоты
исходной камеры. Фильтрация по timestamp происходит до копирования в GstBuffer,
кадры не накапливаются и не дублируются. Камера/IMU/детекторы не замедляются.
При прямом захвате только startup `fps`, поля max_fps нет.
Bitrate указан в бит/с; сейчас его изменение требует остановленного выхода.

### Состояние и RTP

Status дополнительно содержит:
`run_id, ssrc, payload_type=96, clock_rate=90000, encoding_name=H264, mtu=1400,
negotiated_caps, packets, actual_fps, frames_submitted, frames_skipped,
error, simulated, exact_osd=false`.

`state`: stopped / starting / running / failed; при недоступном stream-worker
supervisor возвращает unavailable без сохранения старых RTP-реквизитов.
Каждый настоящий запуск получает новый run_id/SSRC, повторная подписка их не меняет.
`negotiated_caps` появляется после начала передачи, это фактический вход encoder.
`producer.requested` означает включённый дополнительный preview;
`producer.publishing` — производитель уже выдаёт кадры. Это не утверждение,
что кодер запущен или оператор принял/декодировал видео.

События `videostream.started {name,run_id}`,
`videostream.stopped {name,reason}`, `videostream.failed {name,error}`
вспомогательные: после потери события GUI запрашивает status.
Datastream `videostream.state` содержит active_streams:
`[{name, transport, state, dependencies}]`; это не полный каталог.
OSD/datastream не заменяются этой схемой, RTP UnicamSequence пока не реализован.

## Жизненный цикл

GUI сначала готовит UDP-приёмник, затем выполняет subscribe.
Остановленный выход запускается только по запросу владельца; settings можно
передать сразу или заранее применить через update. Рабочие producers не
запускаются автоматически: для camera сначала camera.start, для localisation
ещё localisation.start. Только прямой захват открывается самим stream-worker.

Если выход работает, subscribe **без settings** добавляет получателя.
Для защиты от гонки старта запрос с отличающимися settings даёт settings_conflict,
а с совпадающими не перезапускает encoder. Повтор с тем же портом не создаёт
получателя второй раз; для смены порта сначала unsubscribe.
Разные выходы не могут использовать один и тот же IP:порт.

Уход последнего получателя (unsubscribe, session.close, истечение heartbeat):
кодер/RTP останавливаются; прямой захват освобождает камеру.
Для диагностического выхода producer получает source.stop и больше не рисует,
не копирует и не публикует preview. Рабочая камера и алгоритмы продолжаются.
Stop для всех удаляет подписки; повторный stop безопасен. Status/list ничего
не запускают, прежние получатели сами не возвращаются после stop/failure.
После перезапуска worker или неопределённого исхода команды GUI проверяет
status/subscribed/run_id. Повторы одного UDP request ID дедуплицируются
общим механизмом; новый subscribe сам по себе также идемпотентен.

Supervisor включает source.start только перед первым запуском диагностической
передачи. При ошибке старта снимает запрос, если нет активной передачи.
Fault stream-worker отключает все запрошенные им previews. Fault producer-а
останавливает его выход и зависимые передачи. Закрытие оператора не останавливает
автономную игру и её вычислительные компоненты.

## Ошибки

| Код | Смысл |
| --- | --- |
| not_supported | Старая или неизвестная команда |
| not_found | Воркер не объявил выход с таким именем |
| not_owner | Для запуска/изменения/общей остановки нужна lease |
| not_ready | Producer или его зависимости не готовы |
| settings_conflict | Работающий выход уже использует другие настройки |
| restart_required | Изменение startup-поля работающего выхода |
| invalid_argument | Неверный тип/диапазон, неизвестное или фиксированное поле, совпадение портов |
| conflict | Сессия уже получает этот выход на другом порту |
| camera_busy | Конфликт захвата в stream-worker |
| worker_unavailable | stream-worker недоступен |
| pipeline_error | Не удалось запустить GStreamer |
| busy | Достигнут лимит получателей |

available/reason в каталоге помогают заранее показать camera_busy,
camera_stopped, localisation_not_ready, dependency_not_ready,
worker_unavailable, output_removed или stream_worker_unavailable.
Сам запуск всё равно проверяет условия заново.

## Примеры

Оператор уже имеет lease; камера и синхронная IMU запускаются отдельно:

```text
camera.start {"lease_epoch":1}
videostream.list {"offset":0,"limit":1}
videostream.subscribe {"lease_epoch":1,"name":"camera","rtp_port":5004,"settings":{"fps":60,"max_fps":15,"bitrate":2000000}}
videostream.update {"lease_epoch":1,"name":"camera","settings":{"max_fps":30}}
videostream.status {"name":"camera"}
```

Другой оператор, без lease:

```text
videostream.subscribe {"name":"camera","rtp_port":5004}
videostream.unsubscribe {"name":"camera"}
```

Изменение bitrate влияет на всех; GUI предупреждает об остановке:

```text
videostream.stop {"lease_epoch":1,"name":"camera"}
videostream.update {"lease_epoch":1,"name":"camera","settings":{"bitrate":4000000}}
videostream.subscribe {"lease_epoch":1,"name":"camera","rtp_port":5004}
```

## Камера

Геометрия фиксирована: сенсор **1600x1300 RAW10**, ISP **800x650 BGR**.
`camera.start` всегда включает захват и привязку IMU. Параметра `with_imu` нет;
для ручного видео без IMU используется прямой выход stream-worker.

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
