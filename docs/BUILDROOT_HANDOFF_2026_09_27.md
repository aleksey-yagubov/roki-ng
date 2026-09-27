# Агенту сборки образа

Срез публикации 27.09.2026. Исходники runtime:
[aleksey-yagubov/roki-ng](https://github.com/aleksey-yagubov/roki-ng).
Первый опубликованный код runtime: commit `f162992` (main); последующие
документационные правки не меняют его wire/API. В рецепте образа фиксировать
полный commit выбранной сборки, не плавающий main.

## Согласованное обновление

- DT: убрать AMA2 и DMA на UART CM4, сохранить AMA5 для загрузчика motherboard.
  Это не изменение DMA USART1 в прошивке Зубра.
- В образе нужны iceoryx2 с Python binding и `v4l2-ctl`/`media-ctl`.
- Сохраняются libcamera с UnicamSequence, numpy, MessagePack, Python GI и
  используемые GStreamer plugins. Для LAB-детектора нужен Python OpenCV.
- Для движений нужен пакет python-starkit, для bootstrap pyserial и Python
  gpiod v2; для меню сохранить английскую озвучку и доступ к input-устройству.
- Для планируемой калибровки оптики проверить наличие в Python OpenCV
  `fisheye.calibrate`, `findChessboardCorners`, `cornerSubPix` (calib3d/imgproc).
  Это будущее требование, сама процедура в runtime пока не реализована.

## Roki: перейти на опубликованный fork

Репозиторий: [aleksey-yagubov/roki-mb-interface](https://github.com/aleksey-yagubov/roki-mb-interface).
Закрепить commit **9919dcc5c68bd8beadbe7bbac354af123e77e151**.

В нём уже включены ACM v2, CMake-сборка с системным pybind11, исправления
BodyTimeout/getSinglePos и CALL без resume. Сверить и убрать из рецепта старые
0001/0002/0003 patches, уже вошедшие в fork; не применять порт ACM v2 повторно.
Обновить archive hash после смены SITE/VERSION. Нужные зависимости остались:
rcb4-base-class `af9ade42eddcfa0c3ba0ca28a426d6df50ea0d24` и заголовки
roki-mb-service `0d64f8bb28cfe816595d07cbd3bd15218c5c0982`.

В `roki-buildroot/package/roki-mb-interface/` уже есть
`0003-acm-v2-native-transport.patch`: ACM v2 не нужно портировать заново.
Но в этом патче `RokiRcb4::motionPlay()` ещё заканчивается `return resume()`.
В опубликованном `roki-mb-interface/src/RokiRcb4.cpp` это исправлено: один CALL
и проверка его ACK, без последующего Kondo resume. Иначе реально запущенный
слот сопровождается ложным BodyTimeout и реконнектом.

Правка и регрессионный тест уже в указанном commit. Источник истины для API:
[ACM_V2.md](https://github.com/aleksey-yagubov/roki-mb-interface/blob/9919dcc5c68bd8beadbe7bbac354af123e77e151/ACM_V2.md).
BodyTimeout уже должен иметь корректное имя ошибки; успешный вызов очищает
старую ошибку. `getSinglePos` исправлен в библиотеке, но это не добавляет
DeviceToCom в прошивку Зубра.

Использовать commit и зависимости в рецепте вместо локального override. Не брать
произвольный latest upstream. Roki.so пересобирается под Python/sysroot образа;
не копировать host-библиотеку с ноутбука. Legacy libMotherboard.so новой сборке
не нужна. Рецепт сейчас отмечает лицензию unknown: не подставлять придуманную MIT.

## Прошивка motherboard

Исходники и утилита опубликованы в `roki-mb-firmware`, не в runtime:
commit **74f657680372420526cc697617baea537ceb1c3a**.
[Релиз acm-v2-2026.09.27](https://github.com/aleksey-yagubov/roki-mb-firmware/releases/tag/acm-v2-2026.09.27)
содержит BIN, HEX, flash_motherboard.py и SHA256SUMS. Релиз предварительный:
свежий MinSizeRel BIN собран и проверен host-тестами, повторно не прошивался.
BIN SHA256: `4e02f814be0bd32ac390c4ad007a5767e4d5f1a416a76384bc57de6226f984f5`.
Исходник скрипта: `roki-mb-firmware/tools/flash_motherboard.py`.
Зависимости утилиты: Python gpiod v2 и pyserial. Не брать старый RPi.GPIO-скрипт.

Образ может опционально устанавливать утилиту и проверенный firmware artifact
с SHA256 и указанием совместимого ACM protocol. Прошивка только явной сервисной
операцией, не при загрузке и не при старте supervisor. Кастомный bootloader
не перезаписывать. Пока никаких файлов образа этой запиской не изменено.

## Persistent-данные

Сохранить `/var/lib/roki-ng/` между обновлениями: parameters.json, будущие
camera-profiles и field-profiles. Пакет содержит универсальные defaults;
профиль конкретного робота нельзя вшивать в общий образ или перезаписывать.
Camera profile является каталогом: JSON-метаданные и готовые `.npy`-таблицы/
матрицы. Они сохраняются вместе; пересчитывать таблицы при каждой загрузке
не требуется. Это не замена бинарных калибровок текстовыми JSON-массивами.
В Buildroot проверить, что выбранная схема rootfs/data-раздела действительно
делает этот путь постоянным, а не tmpfs.
