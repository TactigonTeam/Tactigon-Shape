from tactigon_gear import OneFingerGesture, SocketConfig, TSpeech, TwoFingerGesture
from tactigon_gear.tskin import TSkin
from tactigon_gear.tskin_socket import TSkinSocket

from tactigon_shapes.interfaces.interface import Interface
from tactigon_shapes.modules.tskin.models import TSkinConfigWrapper

class TSkinInterface(Interface):
    _config: TSkinConfigWrapper | None = None
    _tskin: TSkin | None = None
    
    config_class = TSkinConfigWrapper

    @staticmethod
    def walk(args, s: TSpeech, level: int = 0, parent: str = "_init_"):
        if level > len(args) - 1:
            args.append(dict())

        if parent not in args[level]:
            args[level][parent] = list(["---"] if level > 0 else [])

        for hw in s.hotwords:
            if hw.word not in args[level][parent]:
                args[level][parent].append(hw.word)
            if s.children:
                for child in s.children.t_speech:
                    TSkinInterface.walk(args, child, level + 1, hw.word)

    @property
    def can_run(self) -> bool:
        return self._config is not None and self._config.config is not None

    @property
    def running(self) -> bool:
        return self._tskin is not None

    @property
    def connected(self) -> bool:
        return self._tskin is not None and self._tskin.connected

    @property
    def tskin(self) -> TSkin | None:
        return self._tskin

    def init_app(self, *args, **kwargs):
        super().init_app(*args, **kwargs)
        self.start()

    def get_shape_blocks(self) -> dict:
        #         data = {
        #     "has_tskin": has_tskin,
        #     "modKeys": self.hotkey_list,
        #     "funckeys": self.function_list,
        #     "gestures": [(g.label, g.gesture) for g in gestures],
        #     "taps": taps,           
        #     "wristOptions": self.wristOptions,
        #     "gripperOptions": self.gripperOptions,
        # }
        return {
            "has_tskin": self.running,
            "taps": self._get_tap_block_config(),
            "gestures": self._get_gesture_block_config()
        }

    def start(self) -> bool:
        if not self._config:
            self._logger.error("TSkin configuration missing.")
            return False

        if not self._config.config:
            self._logger.error("TSkin configuration incomplete.")
            return False
        
        if not self._tskin:
            if self._config.socket:
                self._tskin = TSkinSocket(self._config.config, self._config.socket)
            else:
                self._tskin = TSkin(self._config.config)

        self._tskin.start()

        return True

    def stop(self):
        if self._tskin:
            self._tskin.terminate()

        self._tskin = None

    def get_socket_data(self) -> dict:
        tskin_selector = None
        tskin_connected = None
        tskin_battery = None

        if self._tskin:
            tskin_selector = self._tskin.selector.value if self._tskin.selector else None
            tskin_connected = self._tskin.connected
            tskin_battery = self._tskin.battery

        return {
            "selector": tskin_selector,
            "connected": tskin_connected,
            "battery": tskin_battery
        }

    @staticmethod
    def _get_tap_block_config() -> list:
        return [[e.name.replace("_", " "), e.name] 
                for fin_gesture in (OneFingerGesture, TwoFingerGesture)
                for e in fin_gesture if "TAP" in e.name]

    def _get_gesture_block_config(self) -> list:
        if not self._config or not self._config.config:
            return []

        model_name = self._config.config.gesture_config.name if self._config.config.gesture_config else None

        return [(g.gesture, g.label) for g in next((m.gestures for m in self._config.models if m.name == model_name), [])]