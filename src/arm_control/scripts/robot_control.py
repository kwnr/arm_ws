#!/usr/bin/python3

import rospy
import threading
from arm_msgs.msg import arm_robot_state
from arm_msgs.msg import arm_joint_state
from arm_msgs.msg import arm_dynamixel_state


### Global 변수
state_lock = threading.Lock()
robot_state = arm_robot_state()

d_lock = threading.Lock()
d1 = arm_dynamixel_state()
d2 = arm_dynamixel_state()
d3 = arm_dynamixel_state()
d4 = arm_dynamixel_state()

lj_lock = threading.Lock()
left_j1 = arm_joint_state()
left_j2 = arm_joint_state()
left_j3 = arm_joint_state()
left_j4 = arm_joint_state()
left_j5 = arm_joint_state()
left_j6 = arm_joint_state()
left_j7 = arm_joint_state()
left_j8 = arm_joint_state()

rj_lock = threading.Lock()
right_j1 = arm_joint_state()
right_j2 = arm_joint_state()
right_j3 = arm_joint_state()
right_j4 = arm_joint_state()
right_j5 = arm_joint_state()
right_j6 = arm_joint_state()
right_j7 = arm_joint_state()
right_j8 = arm_joint_state()

def msg_sub_seperate_msgs():
    def processing(data :arm_robot_state) -> None:
        state_lock.acquire()
        robot_state = data
        state_lock.release()

        d_lock.acquire()
        d1, d2, d3, d4 = robot_state.DXL1, robot_state.DXL2, robot_state.DXL3, robot_state.DXL4
        d_lock.release()


        lj_lock.acquire()
        left_j1, left_j2, left_j3, left_j4 = robot_state.L1, robot_state.L2, robot_state.L3, robot_state.L4
        left_j5, left_j6, left_j7, left_j8 = robot_state.L5, robot_state.L6, robot_state.L7, robot_state.L8
        lj_lock.release()


        rj_lock.acquire()
        right_j1, right_j2, right_j3, right_j4 = robot_state.R1, robot_state.R2, robot_state.R3, robot_state.R4
        right_j5, right_j6, right_j7, right_j8 = robot_state.R5, robot_state.R6, robot_state.R7, robot_state.R8
        rj_lock.release()

    rospy.Subscriber("robot_state", arm_robot_state, processing)

def checking_state_msg():
    checking_duration = rospy.Duration(secs=4)
    check_result = False

    t_start = rospy.Time.now()
    t_check = t_start


    while ((t_check - t_start).to_sec() > checking_duration.to_sec()):
        t_check = rospy.Time.now()

        state_lock.acquire()
        left_input_check = True


        state_lock.release()

def dynamixel_controller():
    import dynamixel_sdk

    """
    System Parameter Define
    """
    # Cormidi Lever Parameter
    _deg_min = [5,  5, -1, -1]
    _deg_max = [1009, 1009, 1, 1]

    _track_inf = [0,0]

    _lever_min = [500, 1160, 2310, 2297] 
    _lever_max = [1600, 2260, 2750, 2800]

    # Dynamixel Comm Paramters
    _PortName = ''
    _CommBaudrate = 115200
    _ProtocolVer = 1.0
    _DX_ID = [11,12,13,14]

    # Dynamixel Protocol Parameters
    _ADDR_MX_TORQUE_ENABLE = 24
    _ADDR_MX_GOAL_POSITION = 30
    _ADDR_MX_PRESENT_POSITION = 36

    _TORQUE_ENABLE = 1
    _TORQUE_DISABLE = 0

    _LEN_MX_GOAL_POSITION = 4
    _LEN_MX_PRESENT_POSITION = 4
    """
    System Parameter Define Done
    """

    """
    Inline Function
    """

    def mapping(value, i):
        if value > 1100 :       # Master의 Master의 트랙_L, 트랙_R 값은 deg_max가 1009이므로 1100이상의 쓰레기 값이 들어오면 Dynamixel을 중립 값으로 제어함
            return 1050 if i == 0 else 1738

        sampling = 20       # 계수, 20*deg_range개
        lev_Range = _lever_max[i] - _lever_min[i]
        deg_Range = _deg_max[i] - _deg_min[i]

        ## Master Encoder → Dyanmixel의 비트값으로 매핑
        if i == 0:
            bit = _lever_min[i] + lev_Range // deg_Range * (value - _deg_min[i])
            bit = _lever_min[i] + _lever_max[i] - bit
            _track_inf[0] = value
        elif i == 1:
            bit = _lever_min[i] + lev_Range // deg_Range * (value - _deg_min[i])
            _track_inf[1] = value
        ## Master Encoder → Dyanmixel의 비트값으로 매핑

        elif i == 2:        ## 리프터 > -1이면 리프터가 내려가고 > 1이면 리프터가 올라감
            if value == -1 : bit = _lever_min[i]
            elif value == 1 : bit = _lever_max[i]
            elif value == 0 : bit = 2472        ## 중립값

        elif i== 3:
            ## HPMS (Hydraulic Power Managemnet System)
            if _track_inf[0]<380 or _track_inf[0]>635 or _track_inf[1]<380 or _track_inf[1]>635:        # 트랙_L, 트랙_R의 유효한 움직임이 있을 때
                if value != -1:     ## Master의 출력이 -1(POWER : 0%)가 아닐 때는 항상 중립(출력 50%)을 유지함
                    bit = 2600
                    ##2630 ~ 3220
                else:       ## Master의 출력이 -1(POWER : 0%)일 때는 출력 0%를 유지함
                    bit = _lever_min[i]
            else:       # 트랙이 유효한 움직임이 없을 때 (in Dead_Zone)
                if value == 0:      ## 출력 50%
                    bit = 2600
                elif value == 1:       ## 출력 100%
                    bit = _lever_max[i]
                elif value == -1:       ## 출력 0%
                    bit = _lever_min[i]

        if bit > _lever_max[i] :     ## 쓰레기 값 방어 > Constraint
            bit = _lever_max[i]
        elif bit < _lever_min[i] :
            bit = _lever_min[i]

        return bit

    """
    Inline Function Done
    """

    _status = False

    _ph = dynamixel_sdk.PortHandler(_PortName)
    _ph.openPort()
    _ph.setBaudRate(_CommBaudrate)
    _pah = dynamixel_sdk.PacketHandler(_ProtocolVer)

    _groupWrite = dynamixel_sdk.GroupSyncWrite(_ph, _pah, _ADDR_MX_GOAL_POSITION, _LEN_MX_GOAL_POSITION)
    _groupRead = dynamixel_sdk.GroupSyncRead(_ph, _pah, _ADDR_MX_PRESENT_POSITION, _LEN_MX_PRESENT_POSITION)

    _motorDirection = [0,0,0,0]

    try:
        for id in _DX_ID:
            comm_result, dxl_error = _pah.write1ByteTxRx(_ph, id, _ADDR_MX_TORQUE_ENABLE, _TORQUE_ENABLE)
            if comm_result != dynamixel_sdk.COMM_SUCCESS:
                rospy.logwarn_once(f"Error occur during on enabling DXL ID : {id} = " + str(_pah.getTxRxResult(comm_result)))
            elif dxl_error != 0:
                rospy.logwarn_once(f"Error msg for DXL ID : {id} = " + str(_pah.getRxPacketError(dxl_error)))
        
    except Exception as e:
        rospy.logerr(e + f" | Comm. establishing fail ({_PortName})! check port number.")

    while not rospy.is_shutdown():
        try:
            _groupWrite.addParam()
            pass
        except KeyboardInterrupt as ke:
            rospy.logerr(ke+f" | Closing {_PortName}...")
            _ph.closePort()


if __name__ == '__main__':
    rospy.init_node('arm_control')
    rospy.loginfo("armstrong robot controller node start...") 

    ### robot_state 구독 및 개별 제어 값 메시지 생성
    th_msg_sub_seperate_msgs = threading.Thread(target=msg_sub_seperate_msgs)
    th_msg_sub_seperate_msgs.start()


    ### robot_state 검증 함수 실행


    ### 다이나믹셀 제어 노드 생성


    ### 유압 실린더 전압 제어 노드 생성