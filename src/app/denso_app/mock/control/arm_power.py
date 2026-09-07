"""The ROS side of the two buttons: enable / disable one arm.

Deliberately does NOT touch the CAN bus. Going around controller_manager to
poke motors directly would leave the controllers still believing they own
those joints, and the next command they write would fight whatever we did.
So both directions go through the manager and the manager stays the single
owner of the hardware.

Two levels are available, chosen per arm in config/web.yaml:

  controllers          deactivate the arm's controllers — joints go limp,
                       the hardware interface stays claimed and reporting
  hardware_components  take the interface itself to 'inactive' — the CAN
                       loop for that arm stops entirely

Default is the first. It is reversible in one call and keeps telemetry alive,
which is what you want from a dashboard button.
"""

from __future__ import annotations

from dataclasses import dataclass

from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from lifecycle_msgs.msg import State
from rclpy.node import Node

#: Seconds to wait for controller_manager. It answers in milliseconds when
#: healthy, so anything approaching this means the manager is wedged and the
#: operator needs to be told, not left watching a spinner.
CALL_TIMEOUT_S = 3.0


@dataclass
class CommandResult:
    """Mirrors an HTTP response on purpose — web/ turns it straight into one."""
    ok: bool
    status: int
    detail: str


class ArmPower(Node):
    def __init__(self, arms: dict[str, dict]):
        super().__init__('denso_web_arm_power')
        self._arms = arms
        self._switch = self.create_client(SwitchController,
                                          '/controller_manager/switch_controller')
        self._hw = self.create_client(SetHardwareComponentState,
                                      '/controller_manager/set_hardware_component_state')

    # ---------- public ----------

    def apply(self, side: str, action: str, latched_fault: bool) -> CommandResult:
        arm = self._arms.get(side)
        if arm is None:
            return CommandResult(False, 404, f'no arm named {side!r}')

        # A motor that tripped on temperature reverted to DISABLED and kept the
        # reason in its latched fault. Enabling straight over that would put
        # torque back into a winding that just overheated, so refuse and make
        # the operator clear it deliberately.
        if action == 'enable' and latched_fault:
            return CommandResult(
                False, 409,
                f'{side} arm has a latched fault — clear it before enabling')

        if not self._switch.service_is_ready():
            return CommandResult(False, 503,
                                 'controller_manager is not up')

        ok, detail = self._switch_controllers(arm, action)
        if not ok:
            return CommandResult(False, 500, detail)

        if arm.get('hardware_components'):
            ok, detail = self._set_hardware(arm, action)
            if not ok:
                return CommandResult(False, 500, detail)

        return CommandResult(True, 200, f'{side} arm {action}d')

    # ---------- internals ----------

    def _switch_controllers(self, arm: dict, action: str) -> tuple[bool, str]:
        """The two directions are not mirror images, and must not be.

        Disable stops *every* controller that can command the arm. bringup
        spawns both a trajectory controller and a forward position controller
        per arm, and VR teleop drives the second — stopping only the first
        would answer 200 while the arm kept moving. Only one of them holds
        the command interfaces at a time, so the rest are already inactive:
        BEST_EFFORT, or the call fails on the ones that were never running.

        Enable activates exactly one. They contend for the same command
        interfaces, so asking for several would be refused; and coming back
        up under a controller nobody chose is its own hazard. STRICT here,
        because "the arm is live now" must not be reported on a guess.
        """
        req = SwitchController.Request()
        req.activate_asap = True

        if action == 'enable':
            name = arm.get('default_controller')
            if not name:
                return False, 'no default_controller configured for this arm'
            req.activate_controllers = [name]
            req.strictness = SwitchController.Request.STRICT
        else:
            names = list(arm.get('controllers', []))
            if not names:
                return True, 'no controllers configured'
            req.deactivate_controllers = names
            req.strictness = SwitchController.Request.BEST_EFFORT

        future = self._switch.call_async(req)
        if not self._spin_for(future):
            return False, 'switch_controller timed out'
        res = future.result()
        return (res.ok, 'ok' if res.ok else (res.message or 'switch refused'))

    def _set_hardware(self, arm: dict, action: str) -> tuple[bool, str]:
        target = State.PRIMARY_STATE_ACTIVE if action == 'enable' \
            else State.PRIMARY_STATE_INACTIVE
        label = 'active' if action == 'enable' else 'inactive'

        for component in arm['hardware_components']:
            req = SetHardwareComponentState.Request()
            req.name = component
            req.target_state.id = target
            req.target_state.label = label
            future = self._hw.call_async(req)
            if not self._spin_for(future):
                return False, f'{component}: set_hardware_component_state timed out'
            if not future.result().ok:
                return False, f'{component} refused {label}'
        return True, 'ok'

    def _spin_for(self, future) -> bool:
        """Wait on a service call without owning the executor.

        This node is spun by the shared executor on the ROS thread, so we
        cannot call spin_until_future_complete here — it would try to spin a
        node that is already being spun. Waiting on the future's own event is
        the safe form.
        """
        return future.done() or self._wait_event(future)

    @staticmethod
    def _wait_event(future) -> bool:
        import threading
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(CALL_TIMEOUT_S)
