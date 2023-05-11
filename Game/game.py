#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy

import random # Para simular el funcionamiento. Borrar en la verión definitiva

import state_machine

class Game:
    # 
    def __init__(self):
        self.state_machine = state_machine.StateMachine()

        # Diccionario utilizado para ejecutar la función correspondiente a cada
        # estado.
        self.current_state_dict = {'ST': self.ST_function,
                                   'GP': self.GP_function,
                                   'RT': self.RT_function,
                                   'PT': self.PT_function,
                                   'FN': self.FN_function}
        self.next_event = None

        self.ST_function()

    # --------------------------------------------------------------------------
    # Funciones para el funcionamiento de la máquina de estados.
    # --------------------------------------------------------------------------
    def step(self):
        # Función utilizada para gestionar las entradas, salidas y activaciones
        # de la máquina de estados.
        # Se envía una nueva entrada a la máquina de estados.
        self.state_machine.new_input(self.next_event)
        
        if self.state_machine.current_output >= 0:
             # Si se ha flanqueado una transición en la máquina de estados en
             # este paso, se llama a la función correspondiente
            state = self.state_machine.current_state.name

            self.current_state_dict[state]()
            
        self.next_event = self.select_event()
            
    def select_event(self):
        # Función que selecciona la entrada de la máquina de estados
        state = self.state_machine.current_state.name

        if state == 'ST':
            if 1 == random.randint(1,5):
                return 'Init'
            else:
                return '-1'

        elif state == 'GP':
            if 1 == random.randint(1,5):
                return 'Robot turn'
            elif 2 == random.randint(1,5):
                return 'Player turn'
            else:
                return '-1'

        elif state == 'RT':
            if 1 == random.randint(1,10):
                return 'End turn'
            elif random.randint(1,10) < 5:
                return 'Player turn'
            else:
                return '-1'

        elif state == 'PT':
            if 1 == random.randint(1,10):
                return 'End'
            elif random.randint(1,10) < 5:
                return 'Robot turn'
            else:
                return '-1'

        else:  # state == 'FN'
            return '-1'

    def run(self):
        while(True):
            self.step()
            if self.state_machine.current_state.name == 'FN':
                break
            time.sleep(0.2)

    # --------------------------------------------------------------------------
    # Funciones de cada uno de los estados.
    # --------------------------------------------------------------------------
    # ST ≡‘Start’, GP ≡ ‘Game Preparation’, RT ≡‘Robot Turn’, PT ≡‘Player 
    # Turn’, FN ≡‘Finish’

    def ST_function(self):
        print("ST")

    def GP_function(self):
        print("GP")

    def RT_function(self):
        print("RT")

    def PT_function(self):
        print("PT")

    def FN_function(self):
        print("FN")


if __name__ == '__main__':
    # Nombre del nodo : "Game".
    rospy.init_node("Game")
    print("Nodo Game activo")
    game = Game()
    
    game.run()
