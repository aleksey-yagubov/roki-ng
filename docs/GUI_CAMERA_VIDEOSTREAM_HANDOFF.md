# GUI: упрощённые видеовыходы

Реализовано 02.10.2026 в roki-ng. Ответ на
`roki-ng-operator-gui/docs/VIDEOSTREAM_SIMPLIFICATION_HANDOFF.md`.
Полный wire-контракт: [CAMERA_VIDEOSTREAM_PROTOCOL.md](CAMERA_VIDEOSTREAM_PROTOCOL.md).

## Обязательные изменения

1. GUI и roki-ng обновлять синхронно, без выбора версии и совместимости.
   В UDP-конверте больше нет v; в hello нет versions; из welcome удалён
   capabilities_revision. System.capabilities не содержит revision/videostream_api,
   videostream.capabilities не содержит api_version, data.list не содержит schema.
   Проверять доступные операции и реальные controls, а не номера API.
2. Удалить UI create/destroy, определения с UUID и выбор кодека. Только H.264.
   Не использовать старые start/attach/detach/sources.
3. Каталог брать из `videostream.list`, страницы по одному элементу до
   next_offset=null. Не хардкодить имена: их объявляют воркеры.
   Текущие имена: stream (раньше direct-gst), camera (раньше runtime), localisation.
   Неработающие известные выходы показывать с available/reason.
4. Поля строить по controls, значения брать из settings.
   fixed=true всегда read-only; live=true доступно владельцу во время работы.
   Остальные поля блокируются после запуска. Наблюдателю заблокированы все.
   Отдельные разделы startup/live не нужны.
5. Геометрия camera/localisation фиксирована производителем: 800x650.
   Удалить JPEG, 800x648, автоматический resize/padding. Другой воркер может
   объявить другое разрешение: не предполагать 800x650 для любого выхода.
6. «Смотреть»: сначала подготовить receiver H264/PT96/90000, затем subscribe
   с name и rtp_port. Для остановленного выхода владелец может передать settings.
   Для работающего выхода settings НЕ передавать: используются текущие общие.
7. «Отключиться»: unsubscribe {name}. Общая остановка владельцем — stop {name},
   с предупреждением, если есть другие получатели. Stop снимает все подписки.
8. Изменяемое поле отправлять через update {name,settings:{ключ:значение}}.
   После успеха перечитать status/settings. При settings_conflict обновить
   состояние и показать действующие настройки, не останавливать другого скрытно.
9. Открытие/закрытие dock-панели — только локальная операция. На имя выхода
   один UDP receiver/декодер, несколько локальных просмотров без новых подписок.
10. После reconnect/смены boot_id повторно запросить каталог и status.
    После нового run_id/SSRC очистить старый приём. Не автозапускать остановленное
    видео из-за сохранённых панелей. При неизвестном исходе subscribe проверить
    status.subscribed, повтор той же подписки не создаёт дубль.

## Сообщения

```text
videostream.list {"offset":0,"limit":1}
videostream.status {"name":"camera"}
videostream.subscribe {"lease_epoch":1,"name":"camera","rtp_port":5004,"settings":{"max_fps":15}}
videostream.update {"lease_epoch":1,"name":"camera","settings":{"max_fps":30}}
videostream.unsubscribe {"name":"camera"}
videostream.stop {"lease_epoch":1,"name":"camera"}
```

settings плоский: width, height, fps, max_fps, bitrate; состав брать из controls.
У прямого захвата дополнительно sensor_width/sensor_height/sensor_depth и нет max_fps.
Bitrate — бит/с, меняется только при остановке. FPS/максимум FPS — числовые,
не обязательно целые. max_fps не выше fps и фактической частоты захвата.
Нужно отображать подтверждённые настройки; они общие для всех подписчиков.
Настройки после restart stream-worker возвращаются к дефолтам производителя.

List items: name/title/state/available/reason/settings/controls/receivers/subscribed/producer.
Controls: type, fixed или min/max/choices, live (отсутствие означает false).
Status дополнительно возвращает run_id/ssrc/encoding_name/payload_type/clock_rate/mtu,
negotiated_caps, actual_fps, counters, error, simulated. run_id/SSRC бывают null
до первого запуска. available=false не означает отсутствие выхода из каталога.
producer.requested/publishing и состояние кодера — разные вещи.

События started/stopped/failed содержат name, не stream_id. События могут
потеряться; status — способ сверки. datastream videostream.state содержит
active_streams с name/transport/state/dependencies. OSD и остальные datastream
не изменены. H.264 не является lossless-источником для точной LAB-калибровки.

## Камера и безопасность

Панель camera.* по-прежнему отдельно управляет камерой+IMU и ISP. Подписка
на camera/localisation не запускает их вычисления. Прямой stream-worker сам
захватывает камеру без IMU и освобождает её после последнего получателя.
Последний зритель ушёл — preview и кодер остановлены, автономная игра продолжается.
На робота эту новую схему в данной итерации не отгружали: GUI и runtime нужно
обновить согласованно.

Общий контракт теперь в [OPERATOR_PROTOCOL.md](OPERATOR_PROTOCOL.md), без V1 в имени.
Во внутренних iceoryx2-именах удалён суффикс /v1; при установке обновить и
перезапустить весь runtime, не смешивать старые и новые воркеры.
