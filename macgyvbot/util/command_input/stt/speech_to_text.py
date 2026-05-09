"""Speech-to-text utility wrapper using speech_recognition."""

try:
    import speech_recognition as sr
except ImportError:  # pragma: no cover - runtime dependency check
    sr = None


class SpeechToTextService:
    def __init__(
        self,
        language,
        device_index,
        energy_threshold,
        pause_threshold,
        dynamic_energy,
        ambient_duration,
        phrase_time_limit,
        logger,
    ):
        if sr is None:
            raise RuntimeError(
                'speech_recognition 패키지가 없습니다. '
                'pip install SpeechRecognition PyAudio 후 다시 실행하세요.'
            )

        self._language = language
        self._device_index = int(device_index)
        self._phrase_time_limit = float(phrase_time_limit)
        self._logger = logger

        self._recognizer = sr.Recognizer()
        self._recognizer.energy_threshold = float(energy_threshold)
        self._recognizer.pause_threshold = float(pause_threshold)
        self._recognizer.dynamic_energy_threshold = bool(dynamic_energy)

        self._log_microphones()

        device = self._device_index if self._device_index >= 0 else None
        self._microphone = sr.Microphone(device_index=device)

        with self._microphone as source:
            self._logger('info', f'주변 소음 측정 중 ({ambient_duration:.1f}s)...')
            self._recognizer.adjust_for_ambient_noise(
                source,
                duration=float(ambient_duration),
            )
            self._logger(
                'info',
                f'보정된 energy_threshold={self._recognizer.energy_threshold:.1f}',
            )

        self._stop_listen = None

    def start(self, text_callback):
        self._stop_listen = self._recognizer.listen_in_background(
            self._microphone,
            lambda recognizer, audio: self._on_audio(recognizer, audio, text_callback),
            phrase_time_limit=self._phrase_time_limit,
        )

    def stop(self):
        if self._stop_listen is None:
            return
        try:
            self._stop_listen(wait_for_stop=False)
        except Exception:
            pass
        self._stop_listen = None

    def _on_audio(self, recognizer, audio, text_callback):
        try:
            text = recognizer.recognize_google(audio, language=self._language)
        except sr.UnknownValueError:
            return
        except sr.RequestError as exc:
            self._logger('warn', f'Google STT 요청 실패: {exc}')
            return

        text = (text or '').strip()
        if not text:
            return

        text_callback(text)

    def _log_microphones(self):
        self._logger('info', '=== 마이크 장치 목록 ===')
        try:
            names = sr.Microphone.list_microphone_names()
        except Exception as exc:
            self._logger('warn', f'마이크 목록 조회 실패: {exc}')
            return

        if not names:
            self._logger('warn', '사용 가능한 마이크 장치를 찾지 못했습니다.')
            return

        for idx, name in enumerate(names):
            mark = ' <- 선택됨' if idx == self._device_index else ''
            self._logger('info', f'[{idx}] {name}{mark}')
