"""Small, optional TTS wrapper for MacGyvBot responses.

The service is intentionally best-effort. If the local TTS engine is missing or
fails, it logs a warning and lets the robot command flow continue.
"""

import queue
import shutil
import subprocess
import threading


class TtsService:
    """Asynchronous system-command based TTS service."""

    def __init__(
        self,
        enabled=False,
        engine='auto',
        voice='ko',
        rate=165,
        timeout_sec=8.0,
        logger=None,
    ):
        self._enabled = bool(enabled)
        self._engine_name = str(engine or 'auto').strip()
        self._voice = str(voice or 'ko').strip()
        self._rate = int(rate)
        self._timeout_sec = float(timeout_sec)
        self._logger = logger
        self._queue = queue.Queue()
        self._stop_event = threading.Event()
        self._worker = None
        self._warned_missing = False
        self._warned_failure = False
        self._engine = self._resolve_engine()

    @property
    def enabled(self):
        return self._enabled

    def start(self):
        if not self._enabled:
            return

        if self._engine is None:
            self._warn_missing_engine()
            return

        self._worker = threading.Thread(
            target=self._run,
            name='macgyvbot_tts_worker',
            daemon=True,
        )
        self._worker.start()
        self._log('info', f'TTS 활성화: engine={self._engine}')

    def stop(self):
        if self._worker is None:
            return

        self._stop_event.set()
        self._queue.put(None)
        self._worker.join(timeout=1.0)
        self._worker = None

    def speak(self, text):
        if not self._enabled:
            return

        text = self._normalize_text(text)
        if not text:
            return

        if self._engine is None:
            self._warn_missing_engine()
            return

        self._queue.put(text)

    def _run(self):
        while not self._stop_event.is_set():
            text = self._queue.get()
            if text is None:
                break

            try:
                self._run_tts_command(self._build_command(text))
            except (OSError, subprocess.SubprocessError) as exc:
                if self._engine == 'espeak-ng' and self._voice:
                    try:
                        self._run_tts_command(
                            self._build_command(text, include_voice=False)
                        )
                        self._warn(
                            'espeak-ng voice 설정이 실패해 기본 voice로 TTS를 출력했습니다.'
                        )
                        continue
                    except (OSError, subprocess.SubprocessError) as fallback_exc:
                        self._warn_failure(fallback_exc)
                        continue

                self._warn_failure(exc)

    def _run_tts_command(self, command):
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=self._timeout_sec,
        )
        if completed.returncode != 0:
            detail = (completed.stderr or '').strip()
            if detail:
                raise subprocess.SubprocessError(detail)
            raise subprocess.SubprocessError(
                f'TTS command failed with code {completed.returncode}'
            )

    def _resolve_engine(self):
        requested = self._engine_name.lower()
        if requested == 'auto':
            if shutil.which('espeak-ng'):
                return 'espeak-ng'
            return None

        if requested in ('none', 'off', 'disabled'):
            return None

        if shutil.which(requested):
            return requested

        return None

    def _build_command(self, text, include_voice=True):
        if self._engine == 'espeak-ng':
            command = ['espeak-ng', '-s', str(self._rate)]
            if include_voice and self._voice:
                command.extend(['-v', self._voice])
            command.append(text)
            return command

        return [self._engine, text]

    @staticmethod
    def _normalize_text(text):
        return ' '.join(str(text or '').split())

    def _warn_missing_engine(self):
        if self._warned_missing:
            return
        self._warned_missing = True
        self._log(
            'warn',
            'TTS 엔진을 찾지 못했습니다. Ubuntu 실행컴에서 "sudo apt install espeak-ng" 후 다시 실행하세요. 음성 출력만 건너뜁니다.',
        )

    def _warn_failure(self, exc):
        if self._warned_failure:
            return
        self._warned_failure = True
        self._log('warn', f'TTS 실행 실패. 음성 출력만 건너뜁니다: {exc}')

    def _warn(self, message):
        self._log('warn', message)

    def _log(self, level, message):
        if self._logger is not None:
            self._logger(level, message)
