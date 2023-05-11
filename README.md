# Integracion

Este repositorio contiene los siguientes archivos:
- ROS_link_node.py: Plantilla básica con el código para crear un nodo con publicadores y subscriptores. Ahora es lo más genérico posible, enviando y recibiendo un string, pero se usará en el futuro para comunicar las partes de visión y lógica del juego con el movimiento del brazo.
- 
- Carpeta Game: Contiene el código necesario para 
   - state_machine.py: Implementación de una máquina de Mealy (máquina de estados) para el desarrollo de la partida. Aunque ahora mismo sería suficiente con una máquina de Moore, ya que las salidas dependerán únicamente del estado, se mantiene así por generalidad. El diseño actual de los estados es el que se muestra en la siguiente figura:

![alt text](https://github.com/DomiBotUPM/Integracion/blob/main/maquina_de_estados.png?raw=true)

Así, el estado 0 ("Start") es el inicial en el que permanece el robót hasta que se inicia la aprtida con la aplicación. En el estado 1 ("Game preparation") el robot robará las fichas y las almacenará en la memoria. En el estado 2 ("Robot turn") el robot hará su jugada. En el estado 3 ("Player turn") elrobot esperará a que termine de jugar
     - game.py. Implementación de la máquina de estados anterior en un nodo de ROS (aunque por el momento no publica ni se subscribe a "topics"). Será lo que controle el desarrollo del juego e indique en que momento se encuentra y que debe hacer el robot.
     
     Cuando se flanquea una transición y se llega a cada uno de los estados se ejecuta una función. Esa función contendrá las distintas tareas (de visión, lógica o movimiento del robot) que corresponden a un momento de la partida. Por ejemplo, en "Player turn" el robot únicamente esperará, mientras que la función de "Robot turn" hará que el robot observe el tablero de juego, elija que ficha jugar y la juege (o decida robar) y avisará a la aplicación de que es elturno del jugador (por ejemplo).
     
     Por el momento se ha implementado un funcionamiento de la máquina de estados de demostración (es aleatorio si flanquea una transición o no y la función de cada estado solo hace un "print" con el nombre del estado), pero se puede ejecutar y observar su funcionamiento. 
     
     FALTA POR DISEÑAR E IMPLEMENTAR:
      - Eventos que hacen que se flanquee cada transcición, en atención a la lógica del juego.
      - Funciones que se ejecutan en cada estado
