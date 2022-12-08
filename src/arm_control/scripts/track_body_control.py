import rospy
from dynamixel_sdk import *


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

class trackControl():
    def __init__(self) -> None:
        self._status = False

        self._ph = PortHandler(_PortName)
        self._ph.openPort()
        self._ph.setBaudRate(_CommBaudrate)
        self._pah = PacketHandler(_ProtocolVer)

        self._groupWrite = GroupSyncWrite(self._ph, self._pah, _ADDR_MX_GOAL_POSITION, _LEN_MX_GOAL_POSITION)
        self._groupRead = GroupSyncRead(self._ph, self._pah, _ADDR_MX_PRESENT_POSITION, _LEN_MX_PRESENT_POSITION)

        self._motorDirection = [0,0,0,0]

        try:
            for id in _DX_ID:
                comm_result, dxl_error = self._pah.write1ByteTxRx(self._ph, id, _ADDR_MX_TORQUE_ENABLE, _TORQUE_ENABLE)
                if comm_result != COMM_SUCCESS:
                    rospy.logwarn_once(f"Error occur during on enabling DXL ID : {id} = " + str(self._pah.getTxRxResult(comm_result)))
                elif dxl_error != 0:
                    rospy.logwarn_once(f"Error msg for DXL ID : {id} = " + str(self._pah.getRxPacketError(dxl_error)))
                
        except:
            rospy.logwarn("Comm. establishing fail! check port number.")


