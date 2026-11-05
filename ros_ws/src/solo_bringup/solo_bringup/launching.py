"""Generic simulation process lifecycle callbacks."""
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown


def observed(action, progress, name, initializes=False):
    """Register lifecycle callbacks before executing the process action."""
    progress.register(name, initializes)

    def started(event, context):
        progress.running(name, event.pid)

    def exited(event, context):
        progress.exited(name, event.returncode)

    return [
        RegisterEventHandler(OnProcessStart(target_action=action, on_start=started)),
        RegisterEventHandler(OnProcessExit(target_action=action, on_exit=exited)),
        action,
    ]


def critical(action):
    return RegisterEventHandler(OnProcessExit(
        target_action=action,
        on_exit=[LogInfo(msg='Critical process exited; shutting down stack.'),
                 EmitEvent(event=Shutdown(reason='critical process exited'))],
    ))
