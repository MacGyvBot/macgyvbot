"""Speech-to-text node for macgyvbot.

수업 시간 예제(`/Users/jiho/src/stt_node.py`)와 같은 방식으로
`speech_recognition`의 Google STT를 사용한다.

출력:
- /stt_text: STT 원문 텍스트
- /stt_result: 수업 예제와 호환되는 STT 원문 텍스트
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import speech_recognition as sr
except ImportError:  # pragma: no cover - ROS 실행 환경에서 안내용
    sr = None


class SttNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        if sr is None:
            raise RuntimeError(
                'speech_recognition 패키지가 없습니다. '
                'pip install SpeechRecognition PyAudio 후 다시 실행하세요.'
            )

        self.declare_parameter('language', 'ko-KR')
        self.declare_parameter('device_index', -1)
        self.declare_parameter('energy_threshold', 300.0)
        self.declare_parameter('pause_threshold', 0.8)
        self.declare_parameter('phrase_time_limit', 5.0)
        self.declare_parameter('dynamic_energy', True)
        self.declare_parameter('ambient_duration', 1.0)
        self.declare_parameter('stt_text_topic', '/stt_text')
        self.declare_parameter('compat_topic', '/stt_result')
        self.declare_parameter('publish_compat_topic', True)

        self._language = self.get_parameter('language').value
        self._device_index = int(self.get_parameter('device_index').value)
        self._phrase_time_limit = float(
            self.get_parameter('phrase_time_limit').value
        )
        self._publish_compat = bool(
            self.get_parameter('publish_compat_topic').value
        )

        energy_threshold = float(self.get_parameter('energy_threshold').value)
        pause_threshold = float(self.get_parameter('pause_threshold').value)
        dynamic_energy = bool(self.get_parameter('dynamic_energy').value)
        ambient_duration = float(self.get_parameter('ambient_duration').value)
        stt_text_topic = self.get_parameter('stt_text_topic').value
        compat_topic = self.get_parameter('compat_topic').value

        self._stt_pub = self.create_publisher(String, stt_text_topic, 10)
        self._compat_pub = (
            self.create_publisher(String, compat_topic, 10)
            if self._publish_compat
            else None
        )

        self._recognizer = sr.Recognizer()
        self._recognizer.energy_threshold = energy_threshold
        self._recognizer.pause_threshold = pause_threshold
        self._recognizer.dynamic_energy_threshold = dynamic_energy

        self._log_microphones()

        device = self._device_index if self._device_index >= 0 else None
        try:
            self._microphone = sr.Microphone(device_index=device)
        except Exception as exc:
            self.get_logger().error(f'마이크 열기 실패: {exc}')
            raise

        with self._microphone as source:
            self.get_logger().info(f'주변 소음 측정 중 ({ambient_duration:.1f}s)...')
            self._recognizer.adjust_for_ambient_noise(
                source,
                duration=ambient_duration,
            )
            self.get_logger().info(
                f'보정된 energy_threshold={self._recognizer.energy_threshold:.1f}'
            )

        self._stop_listen = self._recognizer.listen_in_background(
            self._microphone,
            self._on_audio,
            phrase_time_limit=self._phrase_time_limit,
        )

        self.get_logger().info(
            f'STT 준비 완료: language={self._language}, '
            f'device_index={self._device_index}, '
            f'phrase_time_limit={self._phrase_time_limit:.1f}s'
        )
        self.get_logger().info(f'STT 출력 topic: {stt_text_topic}')
        if self._publish_compat:
            self.get_logger().info(f'수업 예제 호환 topic: {compat_topic}')

    def _log_microphones(self):
        self.get_logger().info('=== 마이크 장치 목록 ===')
        try:
            names = sr.Microphone.list_microphone_names()
        except Exception as exc:
            self.get_logger().warn(f'마이크 목록 조회 실패: {exc}')
            return

        if not names:
            self.get_logger().warn('사용 가능한 마이크 장치를 찾지 못했습니다.')
            return

        for idx, name in enumerate(names):
            mark = ' <- 선택됨' if idx == self._device_index else ''
            self.get_logger().info(f'[{idx}] {name}{mark}')

    def _on_audio(self, recognizer, audio):
        try:
            text = recognizer.recognize_google(audio, language=self._language)
        except sr.UnknownValueError:
            return
        except sr.RequestError as exc:
            self.get_logger().warn(f'Google STT 요청 실패: {exc}')
            return

        text = (text or '').strip()
        if not text:
            return

        msg = String()
        msg.data = text

        self._stt_pub.publish(msg)
        if self._compat_pub is not None:
            self._compat_pub.publish(msg)

        self.get_logger().info(f'STT 인식 결과: "{text}"')

    def destroy_node(self):
        stop_listen = getattr(self, '_stop_listen', None)
        if stop_listen is not None:
            try:
                stop_listen(wait_for_stop=False)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
