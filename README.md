Программная система рабочего места оператора БПЛА

Графический интерфейс приложения состоит из нескольких окон, каж-дое из которых отвечает за определенный функционал программной систе-мы.
1) Главное окно:
Является основным окном, через которое происходит большая часть взаимодействия пользователя с системой.
2)	Окно маршрутизации БПЛА:
Предназначено для маршрутизации БПЛА во время моделирования полёта.
3) Окно индикации БПЛА:
Содержит в себе основную индикацию, связанную с БПЛА.
4) Дополнительные окна:
Окна, предоставляющие дополнительный функционал в работе систе-мы.

Главное окно содержит в себе основные элементы управления БПЛА, такие как тумблеры регулирования параметров полета беспилотника, тум-блер переключения режима полета, кнопки для вызова некоторых управля-ющих команд дрона, а также кнопку подключения, по нажатию которой происходит запуск локального сервера. При успешном запуске сервера за-горается индикатор подключения, расположенный над соответствующей кнопкой.
Помимо этого, в главном окне содержит в себе индикаторы батарей БПЛА, подключения к беспилотнику, сетка текущего положения джойстика при его использовании.
Кроме того, главное окно имеет кнопки вызова окон маршрутизации и индикации беспилотника, на которых содержится соответствующая инфор-мация о БПЛА.
Также, главное окно предоставляет отображение текущего местополо-жения БПЛА и его ориентации в пространстве. 
Для этого на главном окне содержатся виджеты карты или вида с каме-ры, и виджет авиагоризонта, также содержащий в себе в явном виде высоту, направление и абсолютную скорость полета.
Переключение карты или вида с камеры происходит с помощью от-дельно вынесенного над виджетом тумблера.
![Внешний вид главного окна](https://github.com/koldun8010/UAVControlSystem/blob/master/mainwindow.png)

Окно маршрутизации содержит в себе таблицу с точками маршрута, которая заполняется и сбрасывается с помощью соответствующих кнопок.
Помимо этого, окно маршрутизации поддерживает возможность зада-ния точек маршрута, путём нажатия на необходимое место на карте данного окна.
При необходимости изменить очередность точек маршрута или уда-лить какую-либо из них это можно сделать путём, соответственно, перетаски-вания строк таблицы за вертикальные заголовки или нажатия на крестик.
Также в данном окне содержится кнопка, реализующая управляющую команду установки маршрута для БПЛА в соответствии с ранее заданными точками.
![Внешний вид окна маршрутизации](https://github.com/koldun8010/UAVControlSystem/blob/master/routeswindow.png)

Окно индикации содержит в себе информацию о заряде батареи БПЛА, а также индикаторы об исправности работы различных систем БПЛА.
Если индикаторы горят зелёным цветом – это обозначает штатную ра-боту без неполадок систем БПЛА.
При наличии неполадок в работе системы индикатор соответствующей системы будет гореть красным.
![Внешний вид окна маршрутизации](https://github.com/koldun8010/UAVControlSystem/blob/master/indicatorswindow.png)

К дополнительным окнам относятся окна подтверждения, вызываемые при нажатии кнопок управления, таких как взлет, посадка и возврат, а также окно с информацией о состоянии БПЛА, вызываемое при нажатии кнопки предполетной подготовки.
Окно подтверждения обеспечивает безопасность работы системы при случайном вызове команды управления.
Окно с информацией предназначено для понимания текущего состоя-ния беспилотника перед взлетом.
![Внешний вид окна маршрутизации](https://github.com/koldun8010/UAVControlSystem/blob/master/acceptwindow.png)
