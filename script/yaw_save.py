import rospy
from std_msgs.msg import Float32
import pandas as pd

class imu_yaw():
    def __init__(self):
        rospy.init_node('get_yaw')
        
        rospy.Subscriber('/imu_yaw', Float32, self.cb_imu)
        self.yaw_data = []  # Yaw 데이터를 저장할 리스트
        
        # ROS 노드가 종료될 때 실행될 함수를 등록합니다.
        rospy.on_shutdown(self.save_yaw_data_to_csv)
        
    def main(self):
        loop_Hz = 50
        rate = rospy.Rate(loop_Hz)
        
        rospy.loginfo('dky_driver: waiting for gps topics')
        rospy.wait_for_message('/imu_yaw', Float32)
        rospy.loginfo('dku_driver: begin')
        
        while not rospy.is_shutdown():
            if hasattr(self, 'yaw'):
                print(self.yaw)
            rate.sleep()
    
    def cb_imu(self, data):
        self.yaw = data.data  # data 필드에서 실제 yaw 값을 추출
        self.yaw_data.append(self.yaw)  # 리스트에 yaw 값을 추가
    
    def save_yaw_data_to_csv(self):
        # pandas DataFrame을 사용하여 yaw 데이터를 저장하고, csv 파일로 출력
        df = pd.DataFrame(self.yaw_data, columns=['Yaw'])
        #df.to_csv('yaw_data.csv', index=False)
        df.to_csv('/home/owner/catkin_ws/src/2023_dku/ref_path/imu.csv', index=False)
        rospy.loginfo('Yaw data saved to yaw_data.csv')
        
if __name__ == '__main__':
    morai_imu = imu_yaw()
    morai_imu.main()
