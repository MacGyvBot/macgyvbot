"""Small, optional TTS wrapper for MacGyvBot responses.

The service is intentionally best-effort. If the local TTS engine is missing or
fails, it logs a warning and lets the robot command flow continue.
"""

import queue
import shutil
import subprocess
import threading
import tempfile


class TtsService:
    """Asynchronous system-command based TTS service."""

    def __init__(
        self,
        enabled=False,
        engine='auto',
        voice='ko',
        rate=165,
        edge_rate='+10%',
        pitch='+8Hz',
        timeout_sec=8.0,
        logger=None,
    ):
        self._enabled = bool(enabled)
        self._engine_name = str(engine or 'auto').strip()
        self._voice = str(voice or 'ko').strip()
        self._rate = int(rate)
        self._edge_rate = str(edge_rate or '+0%').strip()
        self._pitch = str(pitch or '+0Hz').strip()
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
                self._speak_text(text, self._engine)
            except (OSError, subprocess.SubprocessError) as exc:
                if self._try_fallback(text, exc):
                    continue

                self._warn_failure(exc)

    def _speak_text(self, text, engine):
        if engine == 'edge':
            self._speak_with_edge(text)
            return

        if engine == 'espeak-ng':
            try:
                self._run_tts_command(self._build_espeak_command(text))
            except (OSError, subprocess.SubprocessError):
                if not self._voice:
                    raise
                self._run_tts_command(
                    self._build_espeak_command(text, include_voice=False)
                )
                self._warn(
                    'espeak-ng voice 설정이 실패해 기본 voice로 TTS를 출력했습니다.'
                )
            return

        self._run_tts_command([engine, text])

    def _speak_with_edge(self, text):
        player = self._resolve_audio_player()
        if player is None:
            raise subprocess.SubprocessError(
                'edge-tts 재생 도구를 찾지 못했습니다. Ubuntu에서 "sudo apt install ffmpeg" 후 다시 실행하세요.'
            )

        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=True) as audio_file:
            edge_command = [
                'edge-tts',
                '--voice',
                self._edge_voice(),
                '--rate',
                self._edge_rate,
                '--pitch',
                self._pitch,
                '--text',
                text,
                '--write-media',
                audio_file.name,
            ]
            self._run_tts_command(edge_command)
            self._run_tts_command(self._build_play_command(player, audio_file.name))

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
            if shutil.which('edge-tts'):
                return 'edge'
            if shutil.which('espeak-ng'):
                return 'espeak-ng'
            return None

        if requested in ('none', 'off', 'disabled'):
            return None

        if requested == 'edge':
            return 'edge' if shutil.which('edge-tts') else None

        if requested == 'espeak':
            requested = 'espeak-ng'

        return requested if shutil.which(requested) else None

    def _try_fallback(self, text, original_exc):
        if self._engine == 'edge' and shutil.which('espeak-ng'):
            self._warn(f'edge-tts 실행 실패. espeak-ng로 fallback합니다: {original_exc}')
            try:
                self._speak_text(text, 'espeak-ng')
                return True
            except (OSError, subprocess.SubprocessError) as fallback_exc:
                self._warn_failure(fallback_exc)
                return True

        return False

    def _build_espeak_command(self, text, include_voice=True):
        command = ['espeak-ng', '-s', str(self._rate)]
        voice = self._espeak_voice()
        if include_voice and voice:
            command.extend(['-v', voice])
        command.append(text)
        return command

    def _edge_voice(self):
        if self._voice in ('ko', 'ko-KR', 'korean'):
            return 'ko-KR-SunHiNeural'
        return self._voice

    def _espeak_voice(self):
        if self._voice.startswith('ko-KR-') or self._voice in ('korean',):
            return 'ko'
        return self._voice

    @staticmethod
    def _resolve_audio_player():
        if shutil.which('ffplay'):
            return 'ffplay'
        if shutil.which('mpg123'):
            return 'mpg123'
        return None

    @staticmethod
    def _build_play_command(player, audio_path):
        if player == 'ffplay':
            return ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', audio_path]
        if player == 'mpg123':
            return ['mpg123', '-q', audio_path]
        return [player, audio_path]

    @staticmethod
    def _normalize_text(text):
        return ' '.join(str(text or '').split())

    def _warn_missing_engine(self):
        if self._warned_missing:
            return
        self._warned_missing = True
        self._log(
            'warn',
            'TTS 엔진을 찾지 못했습니다. Ubuntu 실행컴에서 "python3 -m pip install edge-tts" 또는 "sudo apt install espeak-ng" 후 다시 실행하세요. 음성 출력만 건너뜁니다.',
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
