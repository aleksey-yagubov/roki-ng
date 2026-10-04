# Проверка накопленных изменений — 2026-10-05

Серия фиксирует накопленные полевые изменения после `f58872d`.
Это взаимосвязанные блоки одного обновления; развёртывать следует всю серию.
Новых изменений поведения при подготовке коммитов не внесено.

## Состав

- Тело: polling IMU и непрерывный yaw, порог важных ошибок, восстановление
  связи и запрос аппаратного reset, завершённые участки движений, калибровки,
  выбор software/controller kick, исходные алгоритмы конечных движений.
- Зрение: проекция центра мяча с радиусом, сцена мяча и ворот,
  диагностический поток, независимые детекторы локализации, точки и штрихи,
  подтверждение визуальной поправки в неподвижном окне.
- Игра: исходные forward/FIRA, Simple/Danger football, Simple goalkeeper,
  тест подхода и удара, kickoff, память ворот и защита от удара в свои,
  прыжковая доводка/обход, восстановление, голосовое меню и GUI takeover.
- Контракты и инструкции оператора.

## Проверки

`python3 -m pytest -q`: **696 passed, 125 skipped, 15 failed** (23.35 s).
`git diff --check`: ошибок нет.

Полный набор сейчас НЕ зелёный. Падающие проверки требуют отдельной
актуализации с сохранением инвариантов безопасности, не простого удаления:

- `test_football_vision.py`: три случая `test_goal_detector_accepts_actual_two_element_images`
  и `test_cropped_bar_cannot_replace_missing_post`. Старые ожидания
  `posts`/`post+bar` и отсутствия результата отличаются от новых
  `opening` и не разрешающей удар цветовой подсказки.
- `test_head_menu.py::test_local_menu_lease_and_worker_completion`:
  ожидает отказ GUI при локальном владельце; теперь предусмотрен перехват.
- `test_simple_football.py`: `test_ready_ball_kicks_immediately_without_map_or_further_alignment`,
  `test_crossing_heading_target_does_not_reverse_turn_jump`,
  `test_lost_ball_at_jump_limit_repositions_instead_of_stalling`,
  `test_shot_log_distinguishes_own_goal_veto_and_foot_alignment`,
  `test_fifteenth_jump_checks_strike_before_reposition`,
  `test_failed_reposition_does_not_reset_attempt_in_place`,
  три случая `test_any_two_goal_elements_without_map`,
  `test_simple_menu_goal_selection_before_entry_does_not_start_motion`.
  Здесь остались прежние модели ворот, доводки и дерева меню.

Не все пропущенные проверки выполнялись в этой среде. Результат не подтверждает
физическую точность движений, обхода мяча или отсутствие ложных детекций.
Изменения в соседнем GUI-репозитории в эту серию не входят; при выпуске нужно
зафиксировать согласованную GUI-сборку отдельно.
