from core.shared_imports import DigitalInputDevice
from core.utilities import debug


class Touch:
    def __init__(self) -> None:
        front_left, front_right, back_left, back_right = 23, 22, 17, 5

        self._fl = DigitalInputDevice(front_left, pull_up=True)
        self._fr = DigitalInputDevice(front_right, pull_up=True)
        self._bl = DigitalInputDevice(back_left, pull_up=True)
        self._br = DigitalInputDevice(back_right, pull_up=True)

        debug(["INITIALISATION", "TOUCH", "✓"], [25, 25, 50])

    def read(self) -> list[int]:
        # pull_up=True means released is 1, pressed is 0 (usually what you want)
        return [int(self._fl.value), int(self._fr.value), int(self._bl.value), int(self._br.value)]

    def close(self) -> None:
        self._fl.close()
        self._fr.close()
        self._bl.close()
        self._br.close()
