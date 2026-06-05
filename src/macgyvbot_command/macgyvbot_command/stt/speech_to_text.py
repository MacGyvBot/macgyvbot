"""Speech-to-text utility wrapper using speech_recognition."""

from __future__ import annotations

from contextlib import contextmanager
import ctypes
import os
import sys

try:
    import speech_recognition as sr
except ImportError:  # pragma: no cover - runtime dependency check
    sr = None


def _noop_alsa_error_handler(filename, line, function, err, fmt):
    return


@contextmanager
def _suppress_native_stderr():
    try:
        stderr_fd = sys.stderr.fileno()
    except (AttributeError, OSError, ValueError):
        stderr_fd = None

    if stderr_fd is None:
        yield
        return

    saved_stderr_fd = None
    devnull_fd = None
    try:
        saved_stderr_fd = os.dup(stderr_fd)
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, stderr_fd)
        yield
    finally:
        if saved_stderr_fd is not None:
            os.dup2(saved_stderr_fd, stderr_fd)
            os.close(saved_stderr_fd)
        if devnull_fd is not None:
            os.close(devnull_fd)


@contextmanager
def _suppress_alsa_errors():
    asound = None
    handler = None
    try:
        asound = ctypes.cdll.LoadLibrary("libasound.so")
        handler = ctypes.CFUNCTYPE(
            None,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_char_p,
        )(_noop_alsa_error_handler)
        asound.snd_lib_error_set_handler(handler)
    except Exception:
        asound = None
        handler = None

    try:
        with _suppress_native_stderr():
            yield
    finally:
        if asound is not None:
            try:
                asound.snd_lib_error_set_handler(None)
            except Exception:
                pass


class SpeechToTextService:
    def __init__(
        self,
        language,
        device_index,
        energy_threshold,
        pause_threshold,
        phrase_threshold,
        non_speaking_duration,
        dynamic_energy,
        ambient_duration,
        phrase_time_limit,
        logger,
    ):
        if sr is None:
            raise RuntimeError(
                "speech_recognition 패키지가 없습니다. "
                "pip install SpeechRecognition PyAudio 후 다시 실행해 주세요."
            )

        self._language = language
        self._device_index = int(device_index)
        self._phrase_time_limit = float(phrase_time_limit)
        self._logger = logger

        self._recognizer = sr.Recognizer()
        pause_threshold = max(0.1, float(pause_threshold))
        non_speaking_duration = max(
            0.05,
            min(float(non_speaking_duration), pause_threshold),
        )
        self._recognizer.energy_threshold = float(energy_threshold)
        self._recognizer.pause_threshold = pause_threshold
        self._recognizer.phrase_threshold = float(phrase_threshold)
        self._recognizer.non_speaking_duration = non_speaking_duration
        self._recognizer.dynamic_energy_threshold = bool(dynamic_energy)

        device = self._device_index if self._device_index >= 0 else None
        with _suppress_alsa_errors():
            self._microphone = sr.Microphone(device_index=device)

        with _suppress_alsa_errors(), self._microphone as source:
            self._logger("info", f"주변 소음 측정 중 ({ambient_duration:.1f}s)...")
            self._recognizer.adjust_for_ambient_noise(
                source,
                duration=float(ambient_duration),
            )
            self._logger(
                "info",
                f"보정 완료 energy_threshold={self._recognizer.energy_threshold:.1f}",
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
            self._logger("warn", f"Google STT 요청 실패: {exc}")
            return

        text = (text or "").strip()
        if not text:
            return

        text_callback(text)
