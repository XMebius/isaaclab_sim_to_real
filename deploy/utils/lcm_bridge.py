import lcm
from deploy.lcm_types import aliengo_leg_cmd_lcmt, aliengo_body_data_lcmt, aliengo_leg_data_lcmt, rc_command_lcmt

class LCMBridge():
    def __init__(self, command, observer) -> None:
        
        lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.command = command
        self.observer = observer

    def send(self, action: aliengo_leg_cmd_lcmt) -> None:
        self.lc.publish("command_for_robot", self._action)

    def receive(self) -> None:
        self.lc.subscribe("body_state_data", self._body_state)
        self.lc.susbcribe("leg_state_data", self._leg_state)
        self.lc.subscribe("rc_command", self._rc_command)
    
    def _body_state(self, channel, data) -> None:
        msg = aliengo_body_data_lcmt.decode(data)
        self.observer.update_body_state(msg)
    
    def _leg_state(self, channel, data) -> None:
        msg = aliengo_leg_data_lcmt.decode(data)
        self.observer.update_leg_state(msg)

    def _rc_command(self, channel, data) -> None:
        msg = rc_command_lcmt.decode(data)
        # TODO: update the command by condition
        self.observer.update_command(msg)
        self.command.update_command(msg)
        
    def _action(self, channel, data) -> None:
        msg = aliengo_leg_cmd_lcmt.decode(data)
        self.observer.update_action(msg)