# Fantasy RTS/RPG (`gd-rtsrpg`)

Проект Unity **6000.5.4f1**, Universal Render Pipeline **17.5.0**. Фэнтезийная RTS/RPG с индивидуальной армией, отдельными тактическими картами, временными базами и непрерывной историей через реинкарнации героя.

**Фактическое состояние:** Stage 00.1 объединён; **Stage 01 реализован и ожидает приёмки**. Единственная сцена — `Assets/Scenes/SampleScene.unity`: наклонная ортографическая RTS-камера, WASD/стрелки, колесо, перетаскивание средней кнопкой, контрольная плоскость и маркер курсора. [Настройки и ручная проверка](Docs/Stage01Camera.md).

## Документация

- [GDD — правила игры](Docs/GDD.md)
- [Decisions — утверждённые решения](Docs/Decisions.md)
- [Architecture — модульный контракт и фактическая реализация](Docs/Architecture.md)
- [Implementation Plan — этапы 00–13](Docs/ImplementationPlan.md)
- [Working Agreement — управление задачами и ревью](Docs/WorkingAgreement.md)
- [AGENTS.md — правила агентов](AGENTS.md)

## Запуск и разработка

Откройте **корень репозитория** в Unity Hub версии `6000.5.4f1`. Конфигурация пакетов в `Packages/manifest.json` и `Packages/packages-lock.json`; сцена `Assets/Scenes/SampleScene.unity`.

Исходные Unity `.meta` файлы нужно хранить в Git, генерируемые `Library/`, `Temp/`, `Obj/`, `Logs/`, `UserSettings/` и файлы IDE — нет. В шаблоне остаются пакеты AI Navigation и Test Framework, но **игровую навигацию строим самостоятельно без NavMesh**, а на текущем этапе **не добавляем автоматизированные тесты и test infrastructure**.

Компиляция и Play Mode для исходного Bootstrap не были подтверждены менеджером в Unity; статический аудит **не равен** запуску редактора.

Каждый Stage выполняется строго в пределах своего Issue и в отдельном PR. Изменения архитектуры предварительно согласовываются с владельцем.
