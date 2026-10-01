# Передача GUI: камера, стримы, просмотры

Контракт реализован в roki-ng 01.10.2026. GUI в этом изменении не редактировался.
Главный документ: [CAMERA_VIDEOSTREAM_PROTOCOL.md](CAMERA_VIDEOSTREAM_PROTOCOL.md).
Оболочка запросов и lease: [OPERATOR_PROTOCOL_V1.md](OPERATOR_PROTOCOL_V1.md).

## Что изменить

1. Заменить все video.* на videostream.*. Совместимости и алиасов нет.
   Поле create теперь source, не backend. Destination в create отсутствует.
2. Панель «Камера»: camera.start/stop/status/capabilities. Только runtime-захват
   1600x1300 RAW10 -> 800x650 BGR, всегда с IMU. Удалить with_imu и выбор геометрии
   здесь. Для старта нужен MANUAL. Во время aligning кадры уже могут поступать;
   синхронизация IMU готова только при synced.
3. ISP: camera.controls.list, отдельные «Применить» (set) и «Сохранить» (save).
   Freeze фиксирует измеренные AE/AWB, но не пишет JSON. Не смешивать requested
   и measured; supported=null до открытия камеры не означает false.
4. «Источники»: явный videostream.sources с пагинацией по 1 элементу. Показывать
   и available=false с причиной. Запрос списка не запускает ничего.
   Источники direct-gst, runtime, localisation. Детектор пока без видеовыхода.
5. «Передачи»: videostream.list с пагинацией по 4, включая stopped/failed.
   Создание/запуск/обновление/stop/destroy разрешать текущему владельцу управления,
   не только создателю стрима. Отдельных прав на видео нет.
6. До start/attach открыть локальный UDP receiver и передать rtp_port.
   Start требует lease и добавляет вызывающего первым получателем.
   Наблюдатель использует attach только к starting/running. IP робот берёт сам.
   Для одной передачи несколько просмотров используют один receiver/decoder.
7. Кнопка подключения/отключения получателя вызывает attach/detach. Создание,
   перенос и закрытие docking-просмотров не отправляют никаких команд роботу.
   Stop является отдельным действием: он отключает всех, не только свой GUI.
8. После остановки и нового start старые attach не восстанавливаются автоматически.
   Смена run_id/SSRC требует очистки очередей приёма/привязки. Stream_id сохраняется
   до destroy/рестарта stream-worker. События не гарантированы: обновлять list/status.
9. Потеря control lease не останавливает просмотр. При потере heartbeat робот
   отсоединяет только эту сессию; когда получателей нет, сам останавливает передачу.
   Последний detach runtime не останавливает камеру/детектор/локализацию.
10. Для runtime/localisation показать изменяемый max_fps: videostream.update
    без перезапуска. Его верхний предел output.fps, который задаётся при create.
    Direct-gst использует только output.fps capture caps, не live max_fps.
    Bitrate меняется update только при stopped; codec/size/source требуют пересоздания.
11. Datastream camera.state теперь действительно про камеру. Для списка активных
    encoder-ов добавлен videostream.state. Подробный videostream.status показывает
    receivers, attached, destination, actual_fps, counters и negotiated_caps.
12. Не ограничивать UI одним H.264. Несколько передач допускаются, но железо
    может отказать при перегрузке. Один поток на несколько операторов использует
    один кодировщик, а не отдельный на каждого.

## Минимальная последовательность

Все примеры обозначают op и body; оболочку, ID, token и lease добавляет клиент.

```text
control.acquire {}
mode.set {"mode":"MANUAL","lease_epoch":1}
camera.start {"lease_epoch":1}
videostream.sources {"offset":0,"limit":1}
videostream.create {"lease_epoch":1,"source":"runtime","output":{"width":800,"height":650,"fps":60},"max_fps":15,"codec":{"name":"h264","bitrate":2000000}}
videostream.start {"lease_epoch":1,"stream_id":"ID ИЗ CREATE","rtp_port":5004}
videostream.status {"stream_id":"ID ИЗ CREATE"}
```

Во второй сессии без захвата управления:

```text
videostream.list {"offset":0,"limit":4}
videostream.attach {"stream_id":"ТОТ ЖЕ ID","rtp_port":5004}
videostream.detach {"stream_id":"ТОТ ЖЕ ID"}
```

## Что пока не обещать в интерфейсе

UnicamSequence RTP extension и новый MessagePack OSD не добавлены, exact_osd=false.
Выход локализации пока размеченный BGR, не поток только примитивов. Нет нового
выхода маски/remap, lossless потока для threshold tuner или JPEG quality control.
Сжатое/уменьшенное изображение нельзя объявлять цветовым эталоном для thresholds.
Нужно отдельно согласовать удаление resize 800x650 -> 640x520 до LAB и совпадение
преобразования LAB на роботе и ПК; в этом патче GUI-преобразования не менялись.

## Проверки для GUI

- Две сессии смотрят одну передачу; закрытие создателя не выключает второго.
- Последний detach выключает encoder и диагностический выход, не вычислитель.
- Смена владельца даёт управление уже созданными передачами.
- Несколько docking-окон не создают несколько декодеров/attach одного стрима.
- Конфликт direct-gst/runtime показан ошибкой, без автоматической остановки камеры.
- max_fps меняется без скачка run_id/SSRC; stop/start меняет их.
- Списки и controls проходят все страницы, в том числе недоступные источники.
- Наблюдатель может смотреть, но не менять ISP или останавливать чужой просмотр.

На стороне runtime это проверяют tests/test_videostream_contract.py и
tests/test_gstreamer.py. Локальные тесты используют software encoder; новый
контракт ещё нужно совместно проверить с GUI на CM4. Новый Buildroot-пакет
не нужен: multiudpsink находится в уже используемом UDP plugin GStreamer.

Состояние проверки на момент передачи: основной прогон до последних дополнений
дал 651 passed. Последний дополнительный прогон дал 21 passed и одну ошибку
в новом test_camera_control_catalog_with_hardware_ranges: тестовый объект
открытой камеры не задаёт duration. Это незавершённая подготовка mock; реальный
camera.prepare задаёт период. После просьбы пользователя завершить только
документацию тесты и код больше не менялись. Финальный зелёный прогон пока
не заявляется; проверка на CM4 с новым API также не выполнена.
