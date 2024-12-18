import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from pyproj import Proj

class GPSToUTMConverter(Node):
    def __init__(self):
        super().__init__('gps_to_utm_converter')
        # GPS 데이터 구독
        self.subscription = self.create_subscription(
            String,  # GPS 데이터 토픽 형식 (JSON 문자열)
            'gps_data',  # 구독할 토픽 이름
            self.gps_callback,  # 콜백 함수
            10  # 큐 크기
        )
        self.get_logger().info('GPS to UTM Converter Node Started.')
        # UTM 설정
        self.utm_proj = Proj(proj="utm", zone=52, ellps="WGS84", south=False)

    def gps_callback(self, msg):
        try:
            # JSON 데이터 파싱
            gps_data = json.loads(msg.data)
            latitude = gps_data['lat']
            longitude = gps_data['lng']

            # 위도/경도 → UTM 변환
            x, y = self.utm_proj(longitude, latitude)
            
            # 결과 출력
            self.get_logger().info(f"Received GPS Data: Latitude={latitude}, Longitude={longitude}")
            self.get_logger().info(f"Converted UTM Coordinates: x={x:.2f}, y={y:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error processing GPS data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSToUTMConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
