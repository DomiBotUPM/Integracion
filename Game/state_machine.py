#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class State(object):
    # Clase estado. Para cada objeto (cada estado), deberá pasarse al
    # constructor, además del nombre del estado, tres listas con las posibles
    # entradas, salidas y siguientes estados, correspondiéndose entre si en
    # función del índice: inputs[0] produce outputs[0] y cambia al estado
    # next_states[0]
    def __init__(self, name, inputs, outputs, next_states):
        try:
            self.name = name  # Nombre del objeto. Coincidirá con el nombre de la instanciación.
            self.state_transition_function = dict(zip(inputs,zip(outputs,next_states)))  # Diccionario para obtener, para cada estado, la salida y el siguiente estado en función de la entrada. Tiene la forma Clave: entrada[i]. Valor: (salida[i], nuevo estado[i]).
        except:
            print("Error: Los argumentos para la creación del estado no son correctos. En primer lugar se debe pasar al constructor el nombre del estado, seguido de las posibles entradas y las salidas, así como los siguientes estados para cada entrada.")
        
class StateMachine(object):
    # Máquina de estados
    def __init__(self):
        # Variables auxiliares para construir states_dict. ST ≡‘Start’, GP ≡
        # ‘Game Preparation’, RT ≡‘Robot Turn’, PT ≡‘Player Turn’, FN ≡‘Finish’
        states = [State('ST',['Init'],[0],['GP']),
                  State('GP',['Robot turn', 'Player turn'],[1, 2],['RT', 'PT']),
                  State('RT',['Player turn', 'End'],[2, 3],['PT', 'FN']),
                  State('PT',['Robot turn', 'End'],[1, 3],['RT', 'FN']),
                  State('FN',[],[],[])]
        
        states_names = [i.name for i in states]
        
        # Atributos de la clase
        self.last_index = 0  # Valor n del estado CPn más alto.
        self.states_dict = dict(zip(states_names,states))  # Diccionario para acceder a los objetos estados a partir de su nombre. Clave: nombre del estado. Valor: objeto estado.
        self.current_state = self.states_dict['ST']  # Estado (el objeto) en el que se encuentra la máquina de estados. La máquina se inicia en el estado 'Start'
        self.current_output = -1  # Salida actual. Tomará valores entre 0 y 3 en momento en que se produce un cambio de estado y -1 el resto del tiempo.
        
    def new_input(self, event):
        # Método para manejar las transiciones entre estados cuando se produce 
        # una nueva entrada.
        # "self.current_state.state_transition_function[event]" es el valor, 
        # (salida, nuevo estado), del diccionario "state_transition_function"
        # desde estado actual para la entrada que se está manejando.
        try:
            # Si se ha previsto que desde el estadoa cual flanquée una transición con la nueva entrada.
            self.current_output = self.current_state.state_transition_function[event][0]  # 0: salida
            self.current_state = self.states_dict[self.current_state.state_transition_function[event][1]]  # 1: nuevo estado
            print("Cambio de estado. Nuevo estado: ", self.current_state.name, ". Nueva salida: ", self.current_output)
        except:
            # La nueva entrada no produce un cambio de estado.
            self.current_output = -1
            print("Sin cambio de estado. Estado: ", self.current_state.name, ". Salida: ", self.current_output)
