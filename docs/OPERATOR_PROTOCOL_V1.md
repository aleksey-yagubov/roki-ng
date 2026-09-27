# Протокол оператора v1: реализованный ручной runtime

Этот документ задаёт точный контракт версии `manual-1`, реализованной в Python.
Он является спецификацией для первого Qt-клиента. Архитектурный документ
[PROTOCOL_AND_MEDIA.md](PROTOCOL_AND_MEDIA.md) описывает также будущие возможности;
их наличие нельзя предполагать по одному номеру `v=1`.

## Подключение и транспорт

Робот слушает IPv4 UDP, порт `8093`. Адрес задаётся оператором явно. Discovery,
broadcast, multicast и автоматического старта видео нет. Для управления, ответов,
логов и datastream используется один UDP socket на каждой стороне. Сервер отвечает
строго на исходный IP и порт. RTP передаётся на отдельно указанный порт того же IP.

MessagePack: строковые ключи, `use_bin_type=True`, `raw=False`. Максимальный UDP
payload: **1200 байт**. Слишком большие входные сообщения отбрасываются. Отправитель
не разбивает MessagePack на IP-фрагменты. Все UDP sockets устанавливают DF; Linux:
`IP_MTU_DISCOVER=IP_PMTUDISC_DO`. Криптографической аутентификации сейчас нет;
session token предотвращает смешение сессий. Подключение рассчитано на сеть робота.

Пример конверта:

```json
{
  "v": 1,
  "kind": "request",
  "session": 123,
  "token": 456,
  "id": 5,
  "sequence": 0,
  "robot_mono_ns": 0,
  "op": "motion.pose",
  "body": {"lease_epoch": 1, "name": "base_stand"}
}
```

| Поле | Тип и смысл |
| --- | --- |
| `v` | Integer, ровно `1` |
| `kind` | `hello`, `welcome`, `request`, `response`, `event`, `sample` |
| `session`, `token` | Integer uint64; 0 в hello; сервер выдаёт ненулевые значения |
| `id` | Integer uint64; >0 у hello/request/response, 0 у event/sample |
| `sequence` | Integer uint64; возрастающий номер event/sample; для request 0 |
| `robot_mono_ns` | Monotonic timestamp сервера; клиент передаёт 0, часы ПК с ним не сравнивает |
| `op` | Имя операции |
| `body` | Map; неизвестные поля известной операции игнорируются |

Qt/Python должен сохранять ID как целое число. В QML/JavaScript нельзя хранить
uint64 в обычном `Number`: транспорт и сравнения ID оставлять в Python, в QML
передавать строку или объект модели.

Hello:

```json
{"v":1,"kind":"hello","session":0,"token":0,"id":1,"op":"hello",
 "body":{"versions":[1],"client_name":"roki-qt","client_instance":"random-per-client-instance"}}
```

`client_instance`: непустая строка до 64 символов. Повтор hello с тем же адресом и
instance возвращает ту же сессию. `welcome` содержит session/token в конверте, а в
body: `robot_id`, `boot_id`, `heartbeat_ms=500`, `session_timeout_ms=2000`,
`drive_timeout_ms=350`, `state`, `capabilities_revision="manual-1"`,
`max_datagram=1200`. Максимум четыре одновременные сессии. Неверный token/endpoint,
повреждённый пакет или отсутствие свободной сессии не вызывают ответа.

Клиент каждые 500 мс вызывает `session.heartbeat {}`. Любой корректный пакет
сессии обновляет её время активности. Через 2000 мс тишины робот отзывает
управление, удаляет видео этой сессии и её подписки. `session.close {}` сначала
возвращает `{"closing":true}`, затем выполняет ту же очистку.

## Ответы, повторы и порядок

Ответ повторяет `id` и `op`:

```json
{"body":{"result":{"accepted":true,"job_id":"32-hex-characters"}}}
```

Либо:

```json
{"body":{"error":{"code":"busy","message":"Motion already running; command discarded","retryable":true}}}
```

Каждый новый request получает возрастающий ID в пределах сессии. Зависимые
операции клиент выполняет последовательно, но heartbeat не должен ждать
долгого запуска камеры. При потере ответа повторить тот же пакет с
тем же ID. Reference client делает пять отправок с интервалом 250 мс
(для camera.start/stop и control.acquire интервал 5 секунд). Timeout
не отменяет операцию: затем запросить status либо повторить исходный ID.

Сервер хранит готовые ответы в окне 128 ID: от `max_received_id - 127` до
`max_received_id` включительно. Не полученный ранее запрос внутри окна
принимается даже после heartbeat с большим ID: UDP может менять порядок пакетов.
Дубликат возвращает сохранённый ответ,
движение повторно не запускается. Пока запрос исполняется, дубликат игнорируется.
После вытеснения из кэша старый ID получает `stale_request`, а не выполняется
заново. Переиспользование исполняемого или закэшированного ID с другой операцией
или телом даёт `id_conflict`. Исполняемый запрос сохраняется до завершения,
даже если окно уже ушло вперёд. Ответ после такого завершения отправляется,
но за пределами окна не кэшируется. Клиент не переотправляет опасную операцию
с новым ID автоматически после `stale_request`.

На сессию разрешены восемь запросов в работе. Переполнение возвращает закэшированный
`busy`: для новой попытки после busy нужен новый ID. Пакеты drive, события и
datastream не подтверждаются и не повторяются. Пропуск sequence обнаруживает
потерю; авторитетное состояние запрашивается через status/snapshot.

Ошибка `too_large` требует уменьшить страницу. Общая пагинация: `offset` integer
>=0, `limit` integer 1..8, ответ `items`, `total`, `next_offset` (integer или null).
У `system.operations` limit до 12, у `log.snapshot` отдельный cursor и limit до 2.

## Lease и режим

| Операция | Аргументы | Результат |
| --- | --- | --- |
| `control.acquire` | `force: bool = false` | `lease_epoch`, `motion_ready`; при force также `stop_confirmed`, `stop_error` |
| `control.release` | `lease_epoch` | `released=true` |
| `mode.set` | `lease_epoch`, `mode: IDLE или MANUAL` | `state` |

Только один оператор владеет control. Второй получает `busy`. Во все изменяющие
состояние requests добавляется `body.lease_epoch`. Исключения: собственные
подписки/log filters/session.close и `video.stop/destroy` для собственного видео.
После отзыва старые команды отвергаются. Внутренний urgent barrier также удаляет
ещё не исполненные команды из normal socket worker-а.

Для кнопки GUI «Забрать управление» отправить `control.acquire {"force":true}`.
Предыдущий владелец (кнопочное меню или другой оператор) немедленно теряет lease;
epoch увеличивается даже при повторном force текущим владельцем. Старому сетевому
оператору отправляется событие `control.revoked {"reason":"operator_takeover"}`.
Кнопочное меню отменяет свой ожидающий запуск и произносит `Operator control`.
Затем urgent-команда control.takeover сбрасывает текущий программный план и
очередь motherboard, отбрасывает ожидающие normal IPC-команды и разрешает
управление новому владельцу. Это жёсткая остановка, без завершения шага.

Force выдаёт lease даже при недоступном motherboard-worker или ошибке остановки.
В таком случае ответ содержит `motion_ready=false`, `stop_confirmed=false` и
`stop_error` с описанием ошибки. Новые движения и тесты блокируются ошибкой
`stop_unconfirmed`; диагностические запросы остаются доступны. После устранения
проблемы нужно повторить force с новым request ID. Обычный acquire не снимает
эту блокировку. Поле motion_ready также есть в system.status.

Подтверждение остановки означает сброс программного плана и очереди motherboard,
не измеренное неподвижное положение серв и не остановку аппаратного слота Зубра.
Без связи с железом гарантировать физическую остановку невозможно. Захват lease
не перезапускает камеру, IMU или процессы. Без force текущий владелец повторно
получает прежний epoch; новый владелец не отбирает управление автоматически.

`MANUAL` разрешает движения, тесты и создание видео. `IDLE` останавливает body
queue, но сохраняет процессы. При потере lease ходьба/циклический тест завершаются
плавно; программный slot/разовая поза прерываются сбросом очереди. Следующий
владелец может получить `busy`, пока предыдущая ходьба ещё завершается.

## Система

| Операция | Аргументы | Результат |
| --- | --- | --- |
| `system.capabilities` | `{}` | revision, modes, body, video_backends, data_topics, future, hardware_slots, simulated |
| `system.operations` | пагинация | Строковые имена поддерживаемых операций |
| `system.status` | `{}` | state, owner (session ID/null), boot_id, workers, counters |
| `system.restart_stream_worker` | lease_epoch | `restarted: stream` |

Worker watchdog: отсутствие heartbeat 5 секунд переводит процесс в fault;
процесс завершается, control отзывается. `system.status` и логи остаются доступны.
Перезапуск stream-worker удаляет все определения видео и возвращает `IDLE`, если
body worker жив. Body worker после аппаратного отказа пока восстанавливается
перезапуском supervisor. Автоматического повторного запуска движения нет.

## Движения и jobs

Все следующие команды, кроме каталогов и status, требуют lease. Новая разовая
команда при активной получает `busy`; очереди будущих прыжков/ударов нет.
Ответ с `job_id` означает **принято**, не «движение закончено».

| Операция | Аргументы без lease_epoch | Результат |
| --- | --- | --- |
| `motion.pose` | `name`: base_stand, crouch, stand, head_field | accepted, job_id |
| `motion.head` | pan [-2666,2666], tilt [-2600,950], frames [1,100], integer | accepted, target {pan,tilt} |
| `motion.slots` | пагинация | Имена программных JSON-slots |
| `motion.slot` | name из каталога, speed_factor [0.25,2], default 1 | accepted, job_id |
| `motion.jump` | direction: forward/backward/left/right/turn_left/turn_right; fraction [0.1,1], default 1 | accepted, job_id |
| `motion.kick` | leg right/left (right), power integer [1,100] (80), offset integer [-60,80] (0) | accepted, job_id |
| `motion.stop_graceful` | `{}` | accepted, job_id текущего движения/null |
| `motion.stop_hard` | `{}` | stopped, pose=unknown, scope=host_body_queue |
| `job.status` | job_id | job_id, operation, status, progress, после завершения reason и pose |
| `job.cancel` | job_id активного job | Как stop_graceful |

Числа в скобках обозначают defaults. В `motion.head` пропущенные pan/tilt сохраняют
последнюю заданную координату, default frames=10. Единицы pan/tilt: ticks модели
ROKI относительно 7500; это не градусы и не измеренные положения.

`base_stand` исполняет исходный Initial_Pose и отдельно ставит оба сустава головы
в ноль. `stand` завершает crouch штатной финальной позой ходьбы; из неизвестной
позы нужен `base_stand`. Повтор уже достигнутой позы не создаёт присед-вставание.
`head_field` берёт tilt из параметра `head.field_tilt`.

Головой можно управлять во время walking/test; команда проходит напрямую через
motherboard forwarding и не задерживается за шагами в STM queue. Во время slot
или разовой позы голова занята ими. `target` означает заданную позицию, без
измерения достижения.

`turn_left/right` выполняют один калибруемый прыжок с долей амплитуды, а не
поворот на измеренное число градусов. Аппаратный hard_kick и запуск произвольных
слотов контроллера пока не входят в API. Mixing-slot запускается worker-ом один
раз перед первой явной командой движения; его номер задаётся `--mixing-slot`
(default 3). Проверка соединения и реконнект сами не запускают mixing.

### Отсутствие и обрыв связи с телом

Motherboard-worker не завершается из-за отсутствия ответа тела. В `motion.state`
публикуется `body_connected`; состояние `degraded` означает
недоступность тела, а не смерть процесса. Запросы списка слотов/тестов и статуса
продолжают обслуживаться.

Событие `body.connection` содержит тот же снимок состояния. Supervisor отправляет
его подключённым операторам; переход также записывается в журнал, доступный через
`log.subscribe`. Потерянное UDP-событие компенсируется подпиской `motion.state`.
Повторные неуспешные проверки без изменения состояния не создают поток логов.

При ошибке текущая задача завершается с `job.failed`, генератор движения,
drive и геометрия походки отбрасываются, поза становится `unknown`.
Worker пытается сбросить очередь STM, затем повторяет подключение с задержкой
1, 2, 4 и максимум 5 секунд. Связь подтверждается тремя последовательными
успешными проверками с интервалом 1 секунда; при доступном теле периодическая
проверка выполняется раз в секунду. Все обращения сериализованы в worker.

Пока тела нет, новые движения получают `body_unavailable` (retryable).
Ответ STM `Busy` означает, что UART занят уже исполняемой командой очереди,
а не потерю связи: клиент получает `body_busy` (retryable), периодический ACK
откладывается на 50 мс без отмены движения. Команда, явно отвергнутая как Busy,
не была отправлена в тело. Движения автоматически не повторяются.
Ошибка кинематики оставляет fault движения; последующие команды получают
`motion_fault`, а не ложный `hardware_error` с реконнектом.
Во время движения worker проверяет счётчик ошибок передачи STM `body_failures`.
При его росте текущая задача завершается ошибкой, остаток очереди сбрасывается.
Нулевой размер очереди не означает успешное выполнение: дополнительно проверяются
этот счётчик, отсутствие активной UART-транзакции и время интерполяции.
После восстановления старые движения не возобновляются. Новые явные команды
доступны без обязательного запроса `base_stand` или `crouch`.
Явно запрошенный test.start сам проверяет положение по IMU тела и
при необходимости выполняет ограниченный подъём перед сценарием теста.
Не следует автоматически повторять команды движения в клиенте после ошибки.

Это не аварийное отключение серв: при оборванном кабеле нельзя гарантировать
остановку уже принятого контроллером движения. Reset подтверждает только сброс
очереди в motherboard, не физическую неподвижность тела.

Ограничение текущей native-библиотеки: на голове без тела наблюдался одиночный
успешный `checkAcknowledge()`. Три проверки фильтруют такой случай, но не заменяют
строгую проверку длины, ACK и checksum ответа внутри Roki/STM. До исправления
native-пути `body_connected` нельзя считать надёжным аппаратным interlock.

### Непрерывная ходьба

Оператор отправляет `kind=sample`, `op=motion.drive`, `id=0` примерно 20 раз/с:

```json
{"v":1,"kind":"sample","session":123,"token":456,"id":0,"sequence":17,
 "op":"motion.drive","body":{"lease_epoch":1,"x":1,"y":0,"yaw":0,
 "speed":0.5,"hold_crouch":true}}
```

`sequence` uint64 возрастает для drive в этой сессии. Повторные и старые samples
отбрасываются. `x/y/yaw` от -1 до 1, speed 0.1..1; default x=y=yaw=0, speed=0.5,
hold_crouch=true. x>0 вперёд, y>0 влево; знак yaw соответствует исходному gait
и требует проверки на роботе перед привязкой к UI. Мёртвая зона 0.05.

Величины умножаются на параметры `motion.max_step_mm`, `motion.max_side_mm`,
`motion.max_yaw_rad` и speed. После 350 мс без свежего drive worker начинает
плавную остановку даже если heartbeat клиента продолжает приходить. При
отпускании клавиш отправляются нули. Перед ходьбой из unknown/stand/base_stand
один раз принимается crouch внутри самой задачи, без отдельной команды клиента.
Явная команда stand из unknown выполняет base_stand, а не финал неизвестной походки.

Плавная остановка дорабатывает цикл и дополнительный терминальный цикл с обеими
ногами на полу. hold_crouch оставляет эту позу, false выполняет stand. Пока
остановка начата, новый drive получает rate-limited событие `motion.rejected`
с error; клиент ждёт завершения job. Задержка здесь обусловлена фазой ходьбы.

**Hard stop ограничен возможностями установленной библиотеки.** Он прекращает
Python-генератор и сбрасывает очередь STM, не отправляет следующие позы. Уже
принятая сервой интерполяция может завершиться. Он не останавливает аппаратный
mixing/другой slot Зубра и не обесточивает сервы. Возвращаемый scope отражает это.
После hard stop поза unknown; ходьба запрещена до recovery. Отказ сброса
возвращается как ошибка, а не `stopped=true`.

### События jobs

`job.progress`: `job_id`, `operation`, `status=running`, `progress` (число
законченных циклов для walking/test, иначе 0). `job.completed`: status
`completed` или `cancelled`, reason (string/null), pose. `job.failed`: status
`failed`, reason, pose=unknown. Хранятся 64 последних job. Потерянное событие
восстанавливается через `job.status`; вытесненный job даёт `not_found`.

Ошибки движения не запускают автоматическое вставание. После устранения причины
успешный hard stop сбрасывает fault, затем можно явно выполнить recovery pose.
Автоматического обнаружения падения в этом минимальном runtime пока нет.

## Тесты

`test.list {}` возвращает `items: [run_test, jump_test, rotation_test, kick_test, get_up_test]`.
`test.describe {name}` возвращает название для UI, параметры, диапазоны,
режимы, требования к IMU и недоступные режимы. Эти запросы ничего не запускают.
`test.start {name, ...параметры, lease_epoch}` возвращает accepted/job_id.
Нужны control lease и MANUAL. Список не рассылается при подключении.

| Тип | Параметры и поведение |
|---|---|
| run_test | mode: short (default), long, spot, backwards, side_left, side_right, custom |
| jump_test | direction: forward (default), backward, left, right, on_spot; count: 1..100, default 10 |
| rotation_test | Общая калибровка прыжковых и шаговых поворотов в обе стороны; дополнительных параметров нет |
| kick_test | mode: regular (default), new_kick; new_kick возвращает not_supported до реализации детектора и аппаратного слота 31 |
| get_up_test | Без дополнительных параметров: проверка положения и один подъём по IMU тела |

Перед каждым test.start, включая kick_test, три чтения IMU тела с интервалом
50 мс должны дать одинаковую классификацию положения. Учтена старая ориентация
датчика на плате: вертикальному телу соответствует +Y датчика, не единичный
кватернион. Для проверки используется направление вертикали, не IMU головы.
Вертикальность допускает наклон до 0.6 рад; слот лёжа выбирается при доминирующей
продольной/боковой компоненте вертикали не менее sin(1 рад). Промежуточные,
перевёрнутые и меняющиеся положения не запускают движение.

Слоты: Roki_2_Get_UP_Stomach, Roki_2_Get_UP_Face_Up, Get_Up_Left, Get_Up_Right.
После одного слота ожидаются завершение очереди/интерполяции, 0.5 с успокоения и
три повторных проверки вертикальности. Повторных попыток нет. Job содержит
initial_posture, recovery_slot (если был подъём), posture_verified и stage.
Состояние motion.state содержит recovering. Ошибки: imu_invalid,
posture_uncertain, posture_unstable, get_up_failed. Следующая часть теста при
ошибке не исполняется. Сам get_up_test не запускает ходьбу и не пишет поправки.

Во время подъёма job.cancel/control.release сбрасывают очередь вместо ожидания
всего слота. Команды головы в этот момент получают busy. Физическую остановку
уже начавшейся интерполяции серв сброс очереди не гарантирует.

RAM IMU тела не содержит timestamp: повторные успешные чтения подтверждают
стабильность возвращаемых значений, но не доказывают свежесть самого датчика.
Оси, работоспособность IMU и движения подъёма необходимо проверить на теле;
на голове без тела это аппаратно не проверяется.

run_test: short выполняет 11 циклов, long/spot/backwards 21 (как в оригинале:
10/20 + дополнительный цикл), side_left/right 20. Шаг вперёд 64 мм, назад
-50 мм, боковой 20 мм; у прямолинейных тестов первые два цикла имеют 1/3 и 2/3
шага. Это калибровочные амплитуды, не ограниченные motion.max_step_mm ручного WASD.
custom принимает cycles (1..100, default 10), step_mm (-64..64, default 24),
side_mm (0..20, default 0), right_leg (bool, default true).
Для остальных режимов эти четыре поля запрещены, чтобы не исказить калибровку.
Тесты ходьбы начинают с walk initial pose и завершаются walk final pose.

Во всех тестах ходьбы и прыжков курс отсчитывается от IMU тела на старте теста.
Ошибка чтения или некорректный кватернион прерывают тест, вместо подстановки
нулевых углов. В jump_test после каждого прыжка выполняется коррекция курса
прыжковыми поворотами (не больше 20 попыток, точность 0,09 рад).
Голова переводится в head.field_tilt для ходьбы/поворотов, -2000 для прыжков.
regular выполняет обычный удар правой ногой с мощностью 100, без камеры.

job содержит test (тип и фактически принятые параметры), progress, stage и
manual_parameters. job.cancel/stop_graceful заканчивают текущий цикл/прыжок/удар,
не начинают следующего и возвращают cancelled. При ходьбе выполняются постановка
ног и final pose. stop_hard очищает очередь без завершения цикла.
Ни отменённые, ни ошибочные тесты не сохраняют автоматическую калибровку.

rotation_test: по 3 полных прыжковых поворота CW/CCW, по 10 шаговых поворотов
с базовым коэффициентом 0,23, проверочные повороты к +120° и обратно.
Yaw разворачивается через границу ±π; нулевые/неправильно направленные измерения
отклоняются. Исправлена опечатка оригинала, изменявшая angle_cw при обработке CCW.
Результаты: motion.jump_yaw_cw/ccw и motion.rotation_yield_right/left.
После движения job переходит в saving, затем supervisor атомарно сохраняет все
четыре ключа в parameters.json и возвращает их worker-у. Только после этого
приходит job.completed с saved=true. Ошибка записи даёт job.failed и saved=false.
Пока идёт сохранение, новые движения и применение поправок запрещены.

Ручной замер после успешно выполненного теста:

```text
test.measure {"job_id":"...","values":{"motion.run_10_mm":1100}}
```

values должен содержать ровно ключи manual_parameters из job. Для short/long это
motion.run_10_mm/run_20_mm (общая дистанция в мм); для боковых тестов
motion.side_left_20_mm/side_right_20_mm (положительная общая дистанция в мм);
для spot это motion.shift_x_mm и motion.shift_y_mm (знаковые смещения всего теста).
Для jump_test это motion.jump_forward_mm/backward_mm/left_mm/right_mm:
средняя дистанция ОДНОГО прыжка, не сумма count прыжков. on_spot, custom,
backwards и kick_test не имеют автоматического сопоставления ручных замеров.
test.measure сохраняет параметры, но не двигает робота. Остальные настройки
по-прежнему можно редактировать через params.set. Файл хранит только supervisor.

Simulation проверяет последовательности и протокол, но неподвижная имитация IMU
не может дать успешную калибровку поворотов. ImuRing этим кодом не реализуется.

## Камера и видео

### Runtime-захват и IMU

`camera.start` открывает отдельный camera-worker для будущей детекции и
локализации. Это не команда отправки RTP оператору. Нужны MANUAL и lease:

```json
{"lease_epoch":1,"with_imu":true,"frame_duration_us":16667,"exposure_us":8000,"gain":1.0}
```

Поля кроме lease необязательны; пример содержит defaults. Sensor фиксирован
на 1600x1300 RAW10, ISP 800x650 BGR. frame_duration_us: целое 8333..100000,
exposure_us: целое 1..frame_duration_us, gain: 1..16. Это границы проверки
аргументов, не обещание поддержки сенсором всех частот. На голове проверены
16667 и 33333 мкс. With_imu=false явно выключает захват стробов STM.

Ответ camera.start подтверждает запуск, **не завершение привязки**.
`camera.status {}` доступен без lease и возвращает running, prepared, frames,
bad_frames, sequence, error, topic и imu_sync. Состояние imu_sync:

- disabled: синхронная IMU не включена или камера остановлена.
- aligning: собираются первые пары timestamps.
- matched: найдено смещение, ожидается подтверждение normal-режима STM.
- synced: переключение подтверждено, смещение можно использовать.

При synced событие `camera.synchronized` содержит unicam_minus_stm, pairs,
max_residual_ns, clock_uncertainty_ns. Оно также доступно через camera.status,
поэтому потеря UDP-события не лишает клиента состояния. Пиксели в iceoryx2
доступны во время aligning, но точный IMU join ещё запрещён.

`camera.stop {"lease_epoch":1}` выключает pipeline и strobe capture/IMU stream.
Остановка допустима не только в MANUAL. Ошибка даёт camera.fault или imu.fault,
останавливает связанный захват и не подставляет выдуманное смещение. Переход
через stop/start заново выполняет привязку. Полный контракт:
[CAMERA_IMU_CAPTURE.md](CAMERA_IMU_CAPTURE.md).

Runtime camera и direct-gst взаимоисключающие; сначала остановить действующий
захват. Datastream camera.state пока описывает stream-worker, а для состояния
runtime camera используется camera.status.

### Прямое видео GStreamer

`camera.capabilities {}` и `video.capabilities {}` не открывают камеру. Ответ
описывает backend и настроенные defaults, не выдаёт их за измеренный каталог
сенсора: `sensor_modes_probed=false`, `settings_verified=false`. В первой версии
реальное probe всех режимов не реализовано. `max_active=1`, `exact_osd=false`,
`rtcp=false`, `live_update=[]`.

Поле capabilities `backends`: `["direct-gst", "runtime"]`. Runtime не
открывает сенсор: сначала выполняется camera.start. В video.create указать
`backend:"runtime"`, codec/output/destination как ниже, **не передавать sensor**.
Выход runtime не больше 800x650; его FPS не может превышать FPS запущенной камеры.
Меньшая частота получается пропуском кадров, а не изменением режима сенсора.
Для RTP/JPEG использовать размер, кратный 8, например 800x648; H.264 поддерживает
800x650. Растяжение выполняет преобразователь GStreamer, а не camera-worker.

Video.stop для runtime не останавливает camera/IMU. Camera.stop останавливает
runtime-видео и детектор перед закрытием камеры; событие video.stopped содержит
reason=camera_stopped. После нового camera.start можно снова video.start с тем
же stream_id. Bitrate H.264 меняется через video.update при остановленном видео.
При всех backend exact_osd=false; OSD не добавляется в RTP.

Создание определения:

```json
{
  "lease_epoch":1,
  "backend":"direct-gst",
  "sensor":{"width":1600,"height":1300,"depth":10},
  "output":{"width":800,"height":650,"fps":60},
  "codec":{"name":"h264","bitrate":2000000},
  "destination":{"rtp_port":5004},
  "mtu":1400
}
```

Все вложенные карты и поля можно опустить: приведены defaults. Sensor depth:
8 или 10; width 320..4096, height 240..4096. Это допустимые запросы, фактическую
поддержку проверяет libcamera при start. Сенсор задаётся явно через sensor-config,
по умолчанию полный `1600x1300 RAW10`; скрытого автовыбора 1280x720 нет.

Output: чётные width 160..1600 и height 120..1300, не больше сенсора; fps 1..120.
Частота запрашивается raw caps GStreamer; физически достижимая частота зависит
от режима сенсора и кодера. Sensor FPS отдельно не задаётся. Bitrate H.264:
100000..20000000 bit/s. JPEG не принимает bitrate; размеры RTP/JPEG кратны 8
(например 800x648). Profile H.264 в данном CM4 pipeline фиксирован high/level4.2.

Destination IP берётся из сессии независимо от полей клиента. rtp_port:
1024..65535. MTU: 576..1400, default 1400 (включая RTP header, без UDP/IP).
До восьми определений видео суммарно, одновременно один активный pipeline.

| Операция | Аргументы помимо lease | Поведение |
| --- | --- | --- |
| `video.create` | Карты из примера | Создаёт stream_id без capture |
| `video.start` | stream_id | Запускает, ответ state=starting; ожидается событие |
| `video.status` | stream_id | Полное определение, state, packets |
| `video.update` | stream_id, bitrate | Только остановленный H.264; активный даёт restart_required |
| `video.stop` | stream_id | NULL pipeline, освобождение камеры; определение сохраняется |
| `video.destroy` | stream_id | Освобождение pipeline и удаление определения |

Определение: `stream_id` (32 hex string), `state`, `spec` (нормализованный create),
`host`, `ssrc` uint32, `payload_type` (H264=96, JPEG=26), `clock_rate=90000`,
`encoding_name`, `exact_osd=false`, `error` string/null.

После первых RTP buffers worker отправляет `video.started` с определением и
`negotiated_caps` для входа кодера. Это показывает фактически согласованный
размер/формат processed stream, не подтверждает RAW metadata. `video.failed`
содержит stream_id и error. GStreamer ERROR/EOS или 10 секунд без RTP buffers
закрывают pipeline, а worker остаётся жив. Повтор start разрешён после устранения
причины. Native output libcamera/GStreamer доступен через logs.

Перед `video.start` клиент открывает UDP receiver и подготавливает pipeline.
Пример H.264 на Linux:

```sh
gst-launch-1.0 udpsrc port=5004 caps='application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000' \
  ! rtpjitterbuffer latency=30 drop-on-latency=true \
  ! rtph264depay ! h264parse ! vah264dec ! waylandsink sync=false
```

В Qt sink заменяется на qml6glsink с привязкой к QQuickItem. JPEG receiver:
`rtpjpegdepay ! jpegparse ! vajpegdec` с caps `encoding-name=JPEG,payload=26`.
На других платформах decoder выбирается клиентом из доступных.

RTP sequence нумерует пакеты, timestamp использует clock 90000, marker означает
границу кадра по соответствующему payload format. В direct-gst сейчас нет
UnicamSequence header extension и точного OSD. Реализация основана на рабочем
`roki-buildroot/.../root/stream-full-fov.sh`; свойства RTP описаны в
[GStreamer rtph264pay](https://gstreamer.freedesktop.org/documentation/rtp/rtph264pay.html),
собственный UDP socket передаётся в
[GstMultiUDPSink](https://gstreamer.freedesktop.org/documentation/udp/multiudpsink.html).

## Запрашиваемые datastream

Доступны `system.workers`, `motion.state`, `camera.state`, `detection.state`.
Это snapshots состояния процессов, не измеренная телеметрия серв/IMU. Body state:
state (ready/fault), pose, active_job, head (заданные ticks), error, simulated.
Camera state: state, active_stream, video_state, packets, simulated.
Также backend, frames_submitted, frames_skipped для runtime-video. Skipped
учитывает видимые читателю пропуски/ограничение FPS, не все возможные потери сети.
Detection state содержит running/profile/frames/error/result/age_ms;
result описывает цветовые области конкретного кадра, не координаты на поле.

### LAB-детектор

| Операция | Аргументы | Результат |
| --- | --- | --- |
| detection.list | {} | detector=colour_blobs, profiles, max_blobs=4, coordinates=image_pixels, classifies_ball=false |
| detection.start | lease_epoch, profile (default orange_ball) | state; нужны MANUAL и работающий camera-worker |
| detection.stop | lease_epoch | state с running=false, result=null |
| detection.status | {} | актуальное состояние, доступно наблюдателю |

Профили: orange_ball, green_field, white_marking, blue_posts, yellow_posts,
white_posts. Это LAB connected components: максимум четыре крупнейшие области
после фильтрации pixels_min и box_area_min. Веса нейросети, проверка мяча на
поле, геометрия ворот и проекция в мировые координаты в этот этап не входят.

В result: frame_sequence, sensor_timestamp_ns, size=[800,650], profile,
blobs, total_blobs, processing_us. Blob: rect=[left,top,right,bottom] с
исключёнными right/bottom, pixels, center=[x,y] (центроид foreground).
Пустой blobs — корректный результат «нет подходящих областей».
После остановки/ошибки result очищается; старый кадр не выдаётся за свежий.

Ключи params: vision.<profile>.l_min/l_max/a_min/a_max/b_min/b_max,
pixels_min, box_area_min. L в 0..100, a/b в -128..127. Min не больше max.
Пример: `params.set {"key":"vision.orange_ball.pixels_min","value":100}`.
Нужна lease; apply=next_frame. Изменяется целостный набор для следующего кадра,
перезапуск камеры/детектора не нужен. Параметры атомарно сохраняются в state-dir;
ошибка записи вызывает откат worker-а к предыдущим значениям.
Начальные пороги — только стартовые значения, не калибровка данного поля.

Это диагностический datastream, **не реализация OSD**. Внутри головы отдельный
iceoryx2 сервис roki/detection/blobs/v1 публикует MessagePack результата размером
до 4096 байт с исходным ID кадра. Для точной геометрии будущая локализация
должна сопоставить этот ID с синхронной IMU, а не использовать последнее измерение.

| Операция | Аргументы | Результат |
| --- | --- | --- |
| `data.list` | `{}` | items: name, kind=state, max_rate_hz=10, schema=1 |
| `data.snapshot` | topic | topic, valid, source_mono_ns, age_ms, data |
| `data.subscribe` | topic, rate_hz [0.2,10], default 2 | subscription_id=topic, rate_hz |
| `data.update` | topic, rate_hz | То же, заменяет подписку |
| `data.unsubscribe` | subscription_id | `{}` |

Одна подписка на topic в сессии. `sample/op=data.sample`:
`subscription`, `sequence` (номер этой подписки), `topic`, `valid`,
`source_mono_ns`, `age_ms`, `data`. Первый snapshot приходит в пределах 50 мс.
Источник обновляется heartbeat worker-а раз в 500 мс; увеличение rate до 10 Гц
не делает его аппаратной телеметрией. Timestamp соответствует получению heartbeat,
а не моменту измерения. `valid=false` после потери worker-а; старые поля могут
оставаться диагностическими и не должны рисоваться как актуальные измерения.

## Логи

Сбор всегда активен; stdout supervisor по умолчанию отключён. Для включения:
`params.set {key:"logging.stdout_enabled",value:true,lease_epoch:...}`.
Захватываются отдельный bounded log socket и stdout/stderr каждого worker,
включая нативные библиотеки. История: последние 1024 записи.

| Операция | Аргументы | Результат |
| --- | --- | --- |
| `log.sources` | пагинация | Имена источников |
| `log.subscribe`, `log.update` | level, sources, after | subscription_id=logs, after |
| `log.unsubscribe` | `{}` | `{}` |
| `log.snapshot` | level, sources, after, limit [1,2] | records, next_after, oldest |

level: DEBUG/INFO/WARNING/ERROR/CRITICAL, default INFO. sources: список точных
имён; пустой означает все. after: номер последней уже полученной записи.
Subscribe без after начинает с новых; after=0 запрашивает доступную историю.
У snapshot default after=0, limit=2. Qt-клиент автоматически подписывается после
hello, независимо от control lease.

`sample/op=log.sample`: `subscription="logs"`, `records`, `dropped`.
Запись: record_sequence, monotonic_ns, source, level, message. Длинный текст
разбивается на записи по 200 UTF-8 байт, одна передача содержит не более двух записей. `dropped`
показывает вытеснение из серверной истории, сетевые потери видны по sequence.
Нативный stdout/stderr получает level INFO, поскольку надёжно определять
severity произвольного текста нельзя. Structured faults идут отдельными events.

В Qt нужны фильтры, поиск и autoscroll по галочке. Пауза виджета не останавливает
сбор логов. При разрыве подписки GUI показывает gap, а не бесконечно копит backlog.

## Параметры

`params.keys {prefix?,offset?,limit?}` возвращает ключи. `params.describe {key}`
возвращает key, type (int/float/bool), default, min/max, apply, description.
`params.get {key}` возвращает key/value. `params.set {key,value,lease_epoch}`
валидирует значение и возвращает key/value/apply. Нет передачи файлов и патчей.

`live` применяется сразу, `next_job` требует отсутствие активного движения,
`restart` сохраняет значение для следующего запуска supervisor. Файл робота:
`<state-dir>/parameters.json`; создаётся из defaults, запись fsync + rename.
Авторитетная schema/defaults находятся в `roki_ng/parameters.py`. Значения являются
стартовыми, а не готовой калибровкой конкретного робота.

## События и ошибки

Кроме jobs/video выдаются `worker.fault` (state/error) и rate-limited
`motion.rejected {error}`. Клиентская библиотека добавляет локальное событие
`client.connection_error`. Events могут теряться: состояние всегда проверяется
status/snapshot. События jobs направляются владельцу control, события видео
создателю stream. Общие faults видят подключённые сессии.

Основные error codes: `invalid_argument`, `not_supported`, `not_found`,
`not_ready`, `not_owner`, `busy`, `invalid_state`,
`hardware_error`, `body_busy`, `motion_fault`, `inverse_kinematics`, `invalid_motion`, `camera_busy`,
`pipeline_error`, `restart_required`, `worker_unavailable`, `worker_timeout`,
`expired`, `cancelled`, `session_expired`, `too_large`, `id_conflict`,
`stale_request`, `internal_error`. Текст message диагностический; UI ветвится по
code, не разбирает сообщение. retryable означает возможность **новой** попытки,
а не разрешение автоматически повторять опасное действие с новым ID.

## Порядок реализации Qt-клиента

1. Hello по введённому IP; heartbeat; log.subscribe; system.capabilities.
2. Пагинированные каталоги operations, motion.slots и params.keys/describe.
3. По Connect Control: control.acquire, затем mode.set MANUAL.
4. Кнопки движений вызывают requests, WASD посылает drive samples; при потере
   фокуса/отпускании всех клавиш посылается zero drive. Не создавать очередь кликов.
5. После video.create настроить receiver, затем video.start; video.started
   подтверждает поток, video.failed показывает причину и сохраняет Stop видимым.
6. Подписки motion.state/camera.state и job.status восстанавливают потерянные события.
7. Disconnect останавливает своё видео, отпускает control и закрывает session.

Готовая Python-реализация транспорта: `roki_ng/client.py`. Её asyncio loop можно
держать в отдельном потоке от Qt GUI; QWidget/QQuickItem изменяются только из
GUI-потока. Это не требует OpenCV на ПК.

## Следующие расширения

Имена game.*, calibration.*, servo.*, osd.*, strategy.*, head_display.* пока
зарезервированы архитектурой и возвращают not_supported. `runtime` camera,
FrameRing/ImuRing и UI физических кнопок уже реализованы; нейроускоритель и
локализация ещё нет. Для новых операций будут добавлены capabilities и schemas.

Будущий video OSD следует [DISPLAY_PROTOCOL.md](DISPLAY_PROTOCOL.md): отдельный
MessagePack sample с bbox/rotated rectangle/circle/text, привязка по UnicamSequence,
рисование средствами Qt6. Локализация на поле, IMU и servo health будут отдельными
requested datastream, не OSD. Direct-gst не требует IMU и не сбрасывает strobe
контейнер ради видео; reset STM counter обязателен при запуске
синхронизированного runtime camera pipeline.
