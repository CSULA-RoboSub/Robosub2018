# my_states.py

from state import State

from CommandPattern import *


class GateState(State):

    def on_event(self, event):
        if event == 'start_gate':
            print('Gate state in process')
            # g_act = Gate_Act()
            # g_act.add(Pregate_Task())
            # g_act.add(Gate_Act())
            # g_act.run()
            return ActivityState()

        return self


class ActivityState(State):

    def on_event(self, event):
        if event == 'start_task':
            print('Activity state in process')
            # act = Activities()
            # act.add(Path_Task())
            # act.add(Dice_Task())
            # act.add(Path_Task())
            # act.add(Slots_Task())
            # act.add(Pinger_A_Task())
            # act.add(Chip_Task())
            # act.add(Roulette_Task())
            # act.add(Pinger_B_Task())
            # act.add(Cash_In_Task())
            # act.run()
            return IdleState()

        return self


class IdleState(State):

    def on_event(self, event):
        if event == 'start':
            print('starting')
            return GateState()

        return self

# End of our states.
