import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped

# 두산 로봇 서비스 메시지 임포트 (패키지 환경에 맞춰 확인 필요)
from dsr_msgs2.srv import MovePause

import math

from macgyvbot.config.config import (
    FORCE_TORQUE_TOPIC,
    FORCE_THRES_2_PAUSE,
)

class CollisionMonitorNode(Node):
    def __init__(self):
        super().__init__('collision_monitor_node')
        
        # 파라미터 선언
        self.declare_parameter('force_torque_topic', FORCE_TORQUE_TOPIC)

        # 서비스 이름 기본값 추가
        self.declare_parameter('pause_service', '/dsr01/motion/move_pause') 
        self.declare_parameter('pause_force_threshold', FORCE_THRES_2_PAUSE) # N 단위 임계치
        
        force_torque_topic = self.get_parameter('force_torque_topic').value
        pause_service = self.get_parameter('pause_service').value
        self.pause_force_threshold = self.get_parameter('pause_force_threshold').value
        
        # pause status flag
        self.is_paused = False 
        
        # Subscriber init
        self.create_subscription(WrenchStamped, force_torque_topic, self._wrench_cb, 10) 

        # Service Client init
        self.pause_client = self.create_client(MovePause, pause_service)
        
        self.get_logger().info(f"Collision Monitor 작동 시작. 임계치 F : {self.pause_force_threshold}N")

        # 서비스 서버가 켜져 있는지 확인 (1초 대기)
        if not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Pause 서비스가 활성화되지 않았습니다! 로봇이 멈추지 않을 수 있습니다.")
            return

    def _wrench_cb(self, msg):
        if self.is_paused:
            # ignore force feedback 
            return

        # parse linear forces
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        # calculate the scalar
        total_force = math.sqrt(fx**2 + fy**2 + fz**2)

        # 충돌 감지 로직 -> 임계치 초과시 pause service call
        if total_force > self.pause_force_threshold:
            self.get_logger().warn(f"💥 충돌 감지! 감지된 힘: {total_force:.2f}N. 일시정지 요청!")

            # pause flag 설정
            self.is_paused = True
            
            # Request 객체 생성 및 비동기 호출
            req = MovePause.Request()
            future = self.pause_client.call_async(req)
            
            # 호출 결과(응답)를 처리할 콜백 연결
            future.add_done_callback(self._pause_response_cb)

    def _pause_response_cb(self, future):
        """서비스 호출 결과를 받아오는 콜백 함수"""
        try:
            response = future.result()

            # dsr_msgs2 의 MovePause 서비스에 보통 success(bool) 필드가 존재함
            if response.success: 
                self.get_logger().info("✅ 로봇 일시정지 성공!")

            else:
                self.get_logger().warn("⚠️ 일시정지 명령은 전달되었으나, 제어기가 거부했습니다.")

        except Exception as e:
            self.get_logger().error(f"서비스 호출 중 예외 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
