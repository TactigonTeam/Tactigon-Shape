import asyncio

from os import path
from bleak import BleakScanner
from tactigon_gear import GestureConfig, Hand, OneFingerGesture, SocketConfig, TSkinConfig, TSpeech, TwoFingerGesture
from tactigon_gear.tskin import TSkin
from tactigon_gear.tskin_socket import TSkinSocket

from tactigon_shapes.interfaces.interface import Interface
from tactigon_shapes.modules.tskin.models import Scorer, TSkinConfigWrapper, TSkinModel


DEFAULT_SCORER_CONFIG = Scorer("shapes", "shapes.scorer", "shapes.json")
DEFAULT_SOCKET_CONFIG = SocketConfig("0.0.0.0")


class TSkinInterface(Interface[TSkinConfigWrapper]):
    _tskin: TSkin | None = None

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

    @property
    def tskin_config(self) -> TSkinConfig | None:
        return self._config.config if self._config else None

    @property
    def tskin_hand(self) -> str | None:
        return self._config.config.hand.value if self._config and self._config.config else None

    def init_app(self, *args, **kwargs):
        super().init_app(*args, **kwargs)
        self.start()

    def get_shape_blocks(self) -> dict:
        return {
            "taps": self._get_tap_block_config(),
            "gestures": self._get_gesture_block_config(),
            "speech": self._get_speech_block_config()
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

    def restart(self):
        self.stop()
        self.start()

    def get_tskin_config(self) -> TSkinConfig | None:
        return self._config.config if self._config else None

    def set_tskin_config(self, mac_address: str, hand: Hand, tskin_name: str, model_name: str):
        if not self._config:
            raise ValueError("TSkin configuration missing.")

        self.stop()
        self._config.config = TSkinConfig(
            address=mac_address, 
            hand=hand, 
            name=tskin_name,
            gesture_config=self.get_gesture_config(hand, model_name)
        )
        self.save_config(self._config)

    def reset_tskin_config(self):
        if not self._config:
            raise ValueError("TSkin configuration missing.")

        self.stop()
        self._config.config = None
        self.save_config(self._config)

    def get_models(self, hand: Hand) -> list[TSkinModel]:
        return [m for m in self._config.models if m.hand.value == hand] if self._config else []

    def get_model(self, hand: Hand, model_name: str) -> TSkinModel | None:
        return next((m for m in self.get_models(hand) if m.name == model_name), None)

    def get_gesture_config(self, hand: Hand, model_name: str) -> GestureConfig | None:
        model = self.get_model(hand, model_name)
        
        if not model:
            return None
        
        return GestureConfig(
            model_path=path.join("models", model.name, "model.pickle"),
            encoder_path=path.join("models", model.name, "encoder.pickle"),
            name=model.name,
            created_at=model.date,
        )

    def get_socket_config(self) -> SocketConfig | None:
        return self._config.socket if self._config else None

    def set_socket_config(self, config: SocketConfig):
        if not self._config:
            raise ValueError("TSkin configuration missing.")
        
        self._config.socket = config
        self.save_config(self._config)

    def get_scorer_config(self, scorer: str) -> Scorer | None:
        return next((s for s in self._config.scorers if s.name == scorer), None) if self._config else None

    def set_scorer_config(self, config: Scorer):
        if not self._config:
            raise ValueError("TSkin configuration missing.")

        existing = next((s for s in self._config.scorers if s.name == config.name), None)
        if existing:
            self._config.scorers.remove(existing)

        self._config.scorers.append(config)
        self._config.selected_scorer = config.name
        self.save_config(self._config)

    def get_selected_scorer(self) -> str | None:
        return self._config.selected_scorer if self._config else None

    def get_scores(self) -> list[str]:
        return [s.name for s in self._config.scorers] if self._config else []
    
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

    def find_tskin_list(self) -> list[dict]:
        async def find_devices():
            devices = await BleakScanner.discover()
            return filter(lambda d: str(d.name).startswith("TSKIN"), devices)

        devices = asyncio.run(find_devices())
        return [{"name": d.name, "id": d.address, "address": d.address} for d in devices]

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

    def _get_speech_block_config(self) -> list:
        if not self._config or not self._config.config:
            return []

        return [(s.name, s.label) for s in self._config.config.speech_config] if self._config.config.speech_config else []