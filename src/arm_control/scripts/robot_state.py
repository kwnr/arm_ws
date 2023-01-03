#!/usr/bin/python3

import rospy
import threading
import numpy as np
from arm_msgs.msg import arm_master_comm
from arm_msgs.msg import arm_robot_state

lock = threading.Lock()

### Global 변수
cmd_input = arm_master_comm()
robot_state = arm_robot_state()

### ROS Parameter 확인하는 함수
def ros_param_chek(robot_state: arm_robot_state):
    rospy.loginfo("Checking predefined config list...")

    gain_p_left = [0] * 8
    gain_i_left = [0] * 8
    gain_d_left = [0] * 8

    gain_p_right = [0] * 8
    gain_i_right = [0] * 8
    gain_d_right = [0] * 8

    prefix_str = '/joint_controller_params/'

    for i in range(8):
        try:
            gain_p_left[i] = rospy.get_param(prefix_str +'L'+ str(i+1) + '/' + 'p')
            gain_i_left[i] = rospy.get_param(prefix_str +'L'+ str(i+1) + '/' + 'i')
            gain_d_left[i] = rospy.get_param(prefix_str +'L'+ str(i+1) + '/' + 'd')

            gain_p_right[i] = rospy.get_param(prefix_str +'R'+ str(i+1) + '/' + 'p')
            gain_i_right[i] = rospy.get_param(prefix_str +'R'+ str(i+1) + '/' + 'i')
            gain_d_right[i] = rospy.get_param(prefix_str +'R'+ str(i+1) + '/' + 'd')

        except:
            rospy.logwarn(f"Joint controller gain is not defined! load default gains... ({i}) (check yaml file)")
            gain_p_left = [0.1, 0.2, 0.2, 0.4, 0.3, 0.15, 0, 0]
            gain_p_right = [0.1, 0.2, 0.2, 0.4, 0.3, 0.15, 0, 0]

            gain_d_left = [0.2, 0.3, 0.3, 0.45, 0.1, 0.1, 0, 0]
            gain_d_right = [0.2, 0.3, 0.3, 0.45, 0.1, 0.1, 0, 0]

            gain_i_left = [0.01] * 8
            gain_i_right = [0.01] * 8
    
    lock.acquire()

    ### P gains
    robot_state.L1.p_gain = gain_p_left[0]
    robot_state.L2.p_gain = gain_p_left[1]
    robot_state.L3.p_gain = gain_p_left[2]
    robot_state.L4.p_gain = gain_p_left[3]
    robot_state.L5.p_gain = gain_p_left[4]
    robot_state.L6.p_gain = gain_p_left[5]
    robot_state.L7.p_gain = gain_p_left[6]
    robot_state.L8.p_gain = gain_p_left[7]

    robot_state.R1.p_gain = gain_p_right[0]
    robot_state.R2.p_gain = gain_p_right[1]
    robot_state.R3.p_gain = gain_p_right[2]
    robot_state.R4.p_gain = gain_p_right[3]
    robot_state.R5.p_gain = gain_p_right[4]
    robot_state.R6.p_gain = gain_p_right[5]
    robot_state.R7.p_gain = gain_p_right[6]
    robot_state.R8.p_gain = gain_p_right[7]

    ### I gains
    robot_state.L1.i_gain = gain_i_left[0]
    robot_state.L2.i_gain = gain_i_left[1]
    robot_state.L3.i_gain = gain_i_left[2]
    robot_state.L4.i_gain = gain_i_left[3]
    robot_state.L5.i_gain = gain_i_left[4]
    robot_state.L6.i_gain = gain_i_left[5]
    robot_state.L7.i_gain = gain_i_left[6]
    robot_state.L8.i_gain = gain_i_left[7]

    robot_state.R1.i_gain = gain_i_right[0]
    robot_state.R2.i_gain = gain_i_right[1]
    robot_state.R3.i_gain = gain_i_right[2]
    robot_state.R4.i_gain = gain_i_right[3]
    robot_state.R5.i_gain = gain_i_right[4]
    robot_state.R6.i_gain = gain_i_right[5]
    robot_state.R7.i_gain = gain_i_right[6]
    robot_state.R8.i_gain = gain_i_right[7]

    # D gains
    robot_state.L1.d_gain = gain_d_left[0]
    robot_state.L2.d_gain = gain_d_left[1]
    robot_state.L3.d_gain = gain_d_left[2]
    robot_state.L4.d_gain = gain_d_left[3]
    robot_state.L5.d_gain = gain_d_left[4]
    robot_state.L6.d_gain = gain_d_left[5]
    robot_state.L7.d_gain = gain_d_left[6]
    robot_state.L8.d_gain = gain_d_left[7]

    robot_state.R1.d_gain = gain_d_right[0]
    robot_state.R2.d_gain = gain_d_right[1]
    robot_state.R3.d_gain = gain_d_right[2]
    robot_state.R4.d_gain = gain_d_right[3]
    robot_state.R5.d_gain = gain_d_right[4]
    robot_state.R6.d_gain = gain_d_right[5]
    robot_state.R7.d_gain = gain_d_right[6]
    robot_state.R8.d_gain = gain_d_right[7]

    lock.release()

    rospy.loginfo("Load controller configs Done...")

    rospy.loginfo("Load connection configs Done...")

### Master 토픽 구독 및 상태 정보에 씌우는 콜백함수
def sub_cmd_input(robot_state: arm_robot_state):
    def cb_cmd_input(data: arm_master_comm) -> None:
        lock.acquire()
        robot_state.input_command = data

        robot_state.L1.goal_pos = data.L1
        robot_state.L2.goal_pos = data.L2
        robot_state.L3.goal_pos = data.L3
        robot_state.L4.goal_pos = data.L4
        robot_state.L5.goal_pos = data.L5
        robot_state.L6.goal_pos = data.L6
        robot_state.L7.goal_pos = data.L7
        robot_state.L8.goal_pos = data.L8

        robot_state.R1.goal_pos = data.R1
        robot_state.R2.goal_pos = data.R2
        robot_state.R3.goal_pos = data.R3
        robot_state.R4.goal_pos = data.R4
        robot_state.R5.goal_pos = data.R5
        robot_state.R6.goal_pos = data.R6
        robot_state.R7.goal_pos = data.R7
        robot_state.R8.goal_pos = data.R8
        lock.release()

    rospy.Subscriber("cmd_input", arm_master_comm, callback=cb_cmd_input)

### Robot State 발행을 위한 함수
def pub_robot_state(robot_state: arm_robot_state):
    rospy.loginfo("Robot state publishing thread is running...")
    pub_rate = rospy.Rate(500)
    robot_state_pub = rospy.Publisher("robot_state", arm_robot_state, queue_size=10)

    while not rospy.is_shutdown():
        lock.acquire()
        robot_state.header.stamp = rospy.rostime.Time.now()
        robot_state_pub.publish(robot_state)
        lock.release()

        pub_rate.sleep()

### CUI 인코더 정보를 받아서 데이터를 Robot State 토픽에 씌우는 함수
def get_encoder_data(robot_state: arm_robot_state):
    rospy.loginfo("Receiving encoder thread is running...")
    import serial
    _serial_port_name = "/dev/ttyUSB1"
    _CUI_ADDR = [b'\x0C', b'\x1C', b'\x2C', b'\x3C', b'\x4C', b'\x5C',        # CUI Encoder 주소
                b'\x6C', b'\x7C', b'\x8C', b'\x9C', b'\xAC', b'\xBC',
                b'\xCC', b'\xDC', b'\xEC', b'\xFC']

    serial_slave_encoder = None

    try:
        rospy.loginfo(f"Trying to open port name : {_serial_port_name}.")
        serial_slave_encoder = serial.Serial(_serial_port_name, 2000000)
        rospy.loginfo(f"{_serial_port_name} port is opened!")
    except:
        rospy.logerr(f"Fail to open {_serial_port_name}. (The encoder serial communications)")
        # return

    ## 슬레이브 Encoder 값을 Degree로 변환
    def getEnc(enc, i):
        deg_min = [-5, -19, -6, -50.5, -122, -60, 0, -66] * 2  #R7 : 368  #R8 : 16383 #L7 : 16348 #L8 : 0
        deg_max = [91, 45, 82, 65, 122, 69, 48.5, -0.5] * 2    #R7 : 1889 #R8 : 13860 #L7 : 14221 #L8 : 2611
        margin = 10
        L_byte = enc[0]             ## 하위 8 비트
        H_byte = enc[1] & 63        ## 상위 8 비트 AND 연산 0011 1111 (==63) <- 왼쪽 2개 비트는 check sum
        H_byte = H_byte << 8        ## bit 연산 왼쪽으로 shift
        if 12<=i<=13:
            return H_byte + L_byte      ## 14비트의 Encoder 값 출력 master의 12 ~ 15번 R7,R8,L7,L8이므로 비트 값을 그대로 씀
        elif 14<=i<=15:
            return 16382 - (H_byte + L_byte)        ## 방향 매핑
        value = (H_byte + L_byte) * 0.021978     ## 14비트의 Encoder 값을 Degree로 변환
        if i == 0 or i ==4 or i == 7 or i == 8 or i == 9 or i == 11:
            value = 360 - value
        if value > 180:
            value = value - 360

        value = round(value, 3)
        return value

    # Slave의 수신 값 오류 검출
    def checkSum(enc):      ## enc = encoder data
        L_byte = bin(enc[0])[2:].zfill(8)
        H_byte = bin(enc[1])[2:].zfill(8)

        k1 = int(H_byte[0])
        k0 = int(H_byte[1])
        total_byte = H_byte[2:] + L_byte
        z1 = int(total_byte[0])
        z0 = int(total_byte[1])
        for i in range(2, 14):
            if i % 2 == 0:
                z1 = z1 ^ int(total_byte[i])
            else:
                z0 = z0 ^ int(total_byte[i])

        if not z1 == k1 and not z0 == k0:
            return True
        else:
            return False

    position = np.array([0.0]*16)
    previous_position = np.array([0.0]*16)

    read_encoder_rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            for i in _CUI_ADDR:
                serial_slave_encoder.write(i)
                input_val = serial_slave_encoder.read(2)
                if not checkSum(input_val):     ## 체크섬에 이상이 잇을 때
                    position[(i[0]-12)//16] = None
                    continue
                position[(i[0]-12)//16] = getEnc(input_val, (i[0]-12)//16) # 수신 데이터를 (i[0]-12)//16번 리스트에 저장

            lock.acquire()

            ### 현재 각도 저장하기 L & R
            robot_state.R1.present_pos = position[0]
            robot_state.R2.present_pos = position[1]
            robot_state.R3.present_pos = position[2]
            robot_state.R4.present_pos = position[3]
            robot_state.R5.present_pos = position[4]
            robot_state.R6.present_pos = position[5]
            robot_state.L1.present_pos = position[6]
            robot_state.L2.present_pos = position[7]
            robot_state.L3.present_pos = position[8]
            robot_state.L4.present_pos = position[9]
            robot_state.L5.present_pos = position[10]
            robot_state.L6.present_pos = position[11]

            robot_state.R7.present_pos = position[12]
            robot_state.R8.present_pos = position[13]
            robot_state.L7.present_pos = position[14]
            robot_state.L8.present_pos = position[15]

            ### 위치 에러 계산하기
            robot_state.R1.error = position[0] - robot_state.R1.goal_pos
            robot_state.R2.error = position[1] - robot_state.R2.goal_pos
            robot_state.R3.error = position[2] - robot_state.R3.goal_pos
            robot_state.R4.error = position[3] - robot_state.R4.goal_pos
            robot_state.R5.error = position[4] - robot_state.R5.goal_pos
            robot_state.R6.error = position[5] - robot_state.R6.goal_pos
            robot_state.L1.error = position[6] - robot_state.L1.goal_pos
            robot_state.L2.error = position[7] - robot_state.L2.goal_pos
            robot_state.L3.error = position[8] - robot_state.L3.goal_pos
            robot_state.L4.error = position[9] - robot_state.L4.goal_pos
            robot_state.L5.error = position[10] - robot_state.L5.goal_pos
            robot_state.L6.error = position[11] - robot_state.L6.goal_pos

            robot_state.R7.error = position[12] - robot_state.R7.goal_pos
            robot_state.R8.error = position[13] - robot_state.R8.goal_pos
            robot_state.L7.error = position[14] - robot_state.L7.goal_pos
            robot_state.L8.error = position[15] - robot_state.L8.goal_pos
            
            ### D-error 계산하기
            robot_state.R1.d_error = position[0] - previous_position[0]
            robot_state.R2.d_error = position[1] - previous_position[1]
            robot_state.R3.d_error = position[2] - previous_position[2]
            robot_state.R4.d_error = position[3] - previous_position[3]
            robot_state.R5.d_error = position[4] - previous_position[4]
            robot_state.R6.d_error = position[5] - previous_position[5]
            robot_state.L1.d_error = position[6] - previous_position[6]
            robot_state.L2.d_error = position[7] - previous_position[7]
            robot_state.L3.d_error = position[8] - previous_position[8]
            robot_state.L4.d_error = position[9] - previous_position[9]
            robot_state.L5.d_error = position[10] - previous_position[10]
            robot_state.L6.d_error = position[11] - previous_position[11]

            robot_state.R7.d_error = position[12] - previous_position[12]
            robot_state.R8.d_error = position[13] - previous_position[13]
            robot_state.L7.d_error = position[14] - previous_position[14]
            robot_state.L8.d_error = position[15] - previous_position[15]
        
            ### I-error 계산하기
            robot_state.R1.i_error = max(-100, min(robot_state.R1.i_error + robot_state.R1.error, 100))
            robot_state.R2.i_error = max(-100, min(robot_state.R2.i_error + robot_state.R2.error, 100))
            robot_state.R3.i_error = max(-100, min(robot_state.R3.i_error + robot_state.R3.error, 100))
            robot_state.R4.i_error = max(-100, min(robot_state.R4.i_error + robot_state.R4.error, 100))
            robot_state.R5.i_error = max(-100, min(robot_state.R5.i_error + robot_state.R5.error, 100))
            robot_state.R6.i_error = max(-100, min(robot_state.R6.i_error + robot_state.R6.error, 100))
            robot_state.L1.i_error = max(-100, min(robot_state.L1.i_error + robot_state.L1.error, 100))
            robot_state.L2.i_error = max(-100, min(robot_state.L2.i_error + robot_state.L2.error, 100))
            robot_state.L3.i_error = max(-100, min(robot_state.L3.i_error + robot_state.L3.error, 100))
            robot_state.L4.i_error = max(-100, min(robot_state.L4.i_error + robot_state.L4.error, 100))
            robot_state.L5.i_error = max(-100, min(robot_state.L5.i_error + robot_state.L5.error, 100))
            robot_state.L6.i_error = max(-100, min(robot_state.L6.i_error + robot_state.L6.error, 100))

            robot_state.R7.i_error = max(-100, min(robot_state.R7.i_error + robot_state.R7.error, 100))
            robot_state.R8.i_error = max(-100, min(robot_state.R8.i_error + robot_state.R8.error, 100))
            robot_state.L7.i_error = max(-100, min(robot_state.L7.i_error + robot_state.L7.error, 100))
            robot_state.L8.i_error = max(-100, min(robot_state.L8.i_error + robot_state.L8.error, 100))

            lock.release()

            previous_position = position

        except Exception as e:
            rospy.logerr(f"Error occured during on get_encoder_data! **RESTART ROBOT STATE NODE** : {e}")
            break
        
        except KeyboardInterrupt as ke:
            rospy.logerr(ke+f" | Closing {_serial_port_name}...")
            serial_slave_encoder.close()

        read_encoder_rate.sleep()



if __name__ == '__main__':
    rospy.init_node('arm_state_monitoring')
    rospy.loginfo("Armstrong robot state node start...")

    ### System Config 파일 받아오는 함수
    th_ros_param_chek = threading.Thread(target=ros_param_chek, args=(robot_state,))
    th_ros_param_chek.start()

    ### Master 조작 신호를 받는 쓰레드 생성
    th_sub_cmd_input = threading.Thread(target=sub_cmd_input, args=(robot_state,))
    th_sub_cmd_input.start()

    ### Robot State 발행하는 쓰레드 생성
    th_pub_robot_state = threading.Thread(target=pub_robot_state, args=(robot_state,))
    th_pub_robot_state.start()

    ### 엔코더 정보 쓰는 쓰레드 생성
    th_get_encoder_data = threading.Thread(target=get_encoder_data, args=(robot_state,))
    th_get_encoder_data.start()

    rospy.spin()