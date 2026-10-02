# FIRA Goalkeeper Implementation Plan

> For agentic workers: execute with subagent-driven-development and review each component.

Goal: implement the existing FIRA_penalty_Goalkeeper menu strategy with observable decisions, bounded lateral movement and explicit stop.
Architecture: synchronized ball observation in detection worker; supervisor-owned game coordinator; body worker owns finite lateral steps; GUI presents start/stop/status. Camera remains sole capture owner.
Tech stack: Python, OpenCV, iceoryx2, asyncio, PySide6/QML.

## Constraints
- No robot loopback changes or network tests on robot.
- No physical movements during implementation/verification.
- Existing manual gait defaults unchanged.
- Initial GUI/game default is observe_only=true. Moving mode requires verified camera geometry parameter and explicit start choice.
- Do not silently implement full field goalkeeper, kicks, dives or automatic get-up.

## Tasks
- [x] Detection: ball.start / ball.stop / ball.status internal operations in detection worker, synchronized raw-frame/IMU subscription. State includes result {valid, reason, frame_sequence, sensor_timestamp_ns, x_m, y_m, rect}, age_ms and running/error. Ground projection with fixed centered head, shape/field/temporal gates. Add deterministic image tests.
- [x] Motion: internal game.step finite side displacement with independent parameters and no repeated unbounded drive. Tests for step bounds, side sign and stop barriers.
- [x] Game: game.start({strategy, observe_only, delay_seconds}), game.stop, game.status plus game.state. Coordinator validates before side effects, owns resource lifecycle, clamps total excursion, stops on stale data/IMU/failure and blocks concurrent manual commands. Unit tests use fake workers on developer computer.
- [x] Menu/GUI: use game lifecycle instead of test job for game items. Add FIRA panel in existing GUI with observe/move selection, status and stop; Qt tests with fake protocol.
- [x] Review: inspect all paths and cancellation races, run relevant tests on developer computer, report actual limitations. Documentation and commits only after verification.

Validation: 52 focused runtime tests passed on developer Mac, 84 GUI tests and QML smoke passed. Process/IPC integration tests require a Linux developer environment; hardware motion remains unverified.

Уточнение от 2026-10-02: отбор мяча приближен к оригинальному цветовому детектору.
Проверки формы и ожидание трёх стабильных наблюдений удалены; окружение может быть
зелёным или белым, из подходящих кандидатов выбирается ближайший. Проверки
свежести сохранены. Текущий алгоритм описан в
[RUNTIME_VIDEO_AND_DETECTION.md](../../RUNTIME_VIDEO_AND_DETECTION.md#игровое-наблюдение-мяча).
