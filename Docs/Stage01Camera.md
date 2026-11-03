# Stage 01 — камера, ввод и picking

Issue: https://github.com/HolyPenguin21/gd-rtsrpg/issues/2

Реализация в исходниках готова к ревью; менеджерская приёмка и проверка в Unity ещё не выполнены. Unity Editor и C# compiler в окружении исполнителя отсутствуют. Статическая проверка не заменяет import, компиляцию или Play Mode. Автотесты и тестовая инфраструктура не добавлялись.

## Запуск

Открыть проект в Unity 6000.5.4f1, затем `Assets/Scenes/SampleScene.unity`, включить Play Mode и дать Game View фокус. WASD/стрелки двигают обзор, колесо изменяет orthographic size, MMB + движение мыши перетаскивает поле. Начальный вид направлен на начало координат. Зелёная сетка имеет шаг 5 world units только для оценки движения, оранжевый маркер показывает пересечение с Y=0. Физические коллайдеры не используются.

## Настройки Main Camera / RtsCameraController

| Поле | Значение сцены / смысл |
|---|---|
| Input Actions | Существующий `Assets/InputSystem_Actions.inputactions`, карта `Camera` |
| Move Speed / Move Smooth Time | 15 units/s / 0.12 unscaled seconds; 0 smoothing = мгновенный отклик |
| Initial Position | (0,0) — мировые X,Z опорной позиции; применяется при Awake |
| Use Bounds / Pan Bounds | Включены; X,Z от −25 до +25; ограничивают опорную позицию, не видимый прямоугольник |
| Tilt / Camera Height | 45° вниз от горизонтали / 25 над опорной плоскостью; yaw фиксирован 0 |
| Plane Height | 0; высота бесконечной математической плоскости picking и опорной позиции |
| Min / Max / Initial Zoom | 4 / 24 / 12 — orthographic size, не FOV |
| Zoom Speed / Smooth Time | 0.02 size units на raw scroll unit / 0.12 unscaled seconds; чувствительность зависит от устройства и ОС, редактируется здесь |
| Show Cursor Marker / Cursor Marker | Включён / отдельная сфера без коллайдера; можно отключить или убрать ссылку |
| Marker Height Offset | 0.03 над точкой picking для видимости |

Изменение Initial Position / Initial Zoom применяется при следующем входе в Play Mode. Другие настройки используются на каждом кадре; OnValidate исправляет отрицательные скорости, перепутанные bounds и min/max. При изменении Plane Height временный Plane автоматически не перемещается: для сравнения подвинуть его Y вручную. Это не контракт будущего рельефа.

## Границы ответственности

`RtsCameraController` → локальная копия карты Camera из существующего Input Actions + Camera + `CursorWorldProjection`. Никаких PlayerInput, EventSystem, managers или второго input asset не добавлено. Input layout редактируется в Input Actions, не в коде. Карты Player/UI сохранены с исходными IDs и bindings; их действия не читаются контроллером. Пересекающиеся WASD/стрелки и MMB/scroll в template/UI сейчас не имеют конкурирующего потребителя в сцене. Политика ввода поверх будущего UI будет отдельной задачей.

`CursorWorldProjection.TryGetCursorWorldPoint(camera, screenPosition, planeHeight, out point)` доступен без компонента или сервиса. Его выход сбрасывается в default перед проверкой; false нельзя трактовать как новую мировую точку. Он отказывает при отсутствующей камере, нечисловых значениях, указателе вне camera pixelRect, параллельном луче, отрицательном расстоянии и невалидном результате. Контроллер дополнительно проверяет focus/доступность ввода. Вызов после обновления камеры даёт актуальную точку.

## Выполненная статическая проверка

- Проверены source/config: одна Camera map в исходном asset, нормализация Pan; Player/UI и control schemes не изменены.
- Проверены код и формулы: XZ-only pan, точный интеграл экспоненциальной скорости за кадр, unscaled time, мгновенная остановка по отпусканию, clamp reference и обеих zoom-величин; MMB без накопленного delta до клика.
- Проверены ветви picking: выход default при false, математическая Plane без Physics/NavMesh и сохранённой старой точки.
- Проверены YAML/JSON, локальные fileID и новые GUID/.meta, связи Camera → controller/input/marker и materials → shader. Существующие URP camera data, light/volume, GUID всех исходных assets, Packages и ProjectSettings сохранены.
- `git diff --check` выполнен. Компиляция C# и shader при этом не запускалась.

## Ручная приёмка в Unity — все пункты NOT RUN

| Проверка | Что выполнить | Статус |
|---|---|---|
| Импорт / C# / shader compilation | Открыть проект и сцену; дождаться импорта; проверить отсутствие ошибок и корректный цвет сетки/маркера | NOT RUN |
| Ортографическая камера | Проверить Camera/Orthographic, наклон, неизменный угол при pan/zoom; изменить tilt в Inspector | NOT RUN |
| WASD / стрелки / диагонали | Проверить направления, одинаковую скорость диагонали/оси после разгона, остановку при отпускании | NOT RUN |
| Сглаживание / частота кадров | Сравнить удержание на низкой и высокой FPS, попробовать smooth time 0 и 0.3 | NOT RUN |
| MMB | Начать/закончить drag без скачка, повторить при min/max zoom; во время MMB клавиатура не двигает камеру | NOT RUN |
| Zoom | Плавность, ограничения 4–24, смена Inspector min/max во время движения; размер и target не выходят за пределы | NOT RUN |
| Bounds | Дойти до каждой границы и углов; отпустить/нажать обратное направление; отключить Use Bounds и выйти за область | NOT RUN |
| Picking | Сравнить маркер и курсор в центре/углах, при pan/разных zoom/tilt; конечный Plane не ограничивает математический picking | NOT RUN |
| Отказ picking | В IDE debugger вызвать static helper с null camera, Camera с горизонтальным лучом и плоскостью позади луча: false и default output. В Play Mode Camera Height=0.1 при near clip=0.3 даёт отрицательную дистанцию для центрального луча: маркер скрывается; затем восстановить 25 | NOT RUN |
| Тактическая пауза | Во время Play Mode в Project Settings → Time установить Time Scale=0, проверить pan, zoom, MMB и picking; затем восстановить 1 | NOT RUN |
| Focus / lifecycle / отпускание | Alt-Tab при удержании клавиш/MMB, вернуться; disable/enable controller; проверить отсутствие залипания/скачка, скрытие маркера при disable | NOT RUN |
| Целостность сцены | Нет missing scripts, ссылки Input Actions и URP/volume целы; Console без ошибок | NOT RUN |

Нет unit selection, orders, Terrain Builder/Procedural Map, navigation, combat, сохранений, edge-scroll, вращения, gamepad controls или runtime rebinding. Следующий функциональный Stage — 02, Terrain Builder, после приёмки 01 и согласования размеров блоков.
