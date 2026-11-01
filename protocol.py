"""2027 single-arm robot serial protocol. Units at the API: m/s, rad/s, mm, rad."""
from dataclasses import dataclass
import math
import struct

JOINT_LIMITS = ((0., 2.5), (0., 5.), (0., 5.), (-6.28, 6.28))
ARM_PROTOCOL_CARTESIAN = 0xC2
ARM_STATUS_TEXT = {0:'正常（不代表到位）',1:'未知机械臂模式，保留上一目标',
                   2:'坐标无合法逆解，保留上一目标',3:'关节反馈尚未准备，等待重试'}


def bounded(value, lower, upper):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('数值必须是有限数字')
    return min(upper, max(lower, value))


@dataclass(frozen=True)
class Command:
    vx: float = 0.
    vy: float = 0.
    wz: float = 0.
    front_mm: float = 30.
    back_mm: float = 30.
    fast_lift: bool = False
    joints: tuple = (0., 0., 0., 0.)
    suction: int = 0
    arm_mode: int = 0  # byte28: joint / XY+phi / XY automatic orientation
    cartesian: tuple = (0., 0., 0.)  # mm, mm, rad; ignored in joint mode
    arm_rate: float = .3  # rad/s, MCU coordinate-mode joint slew rate

    def encode(self):
        if len(self.joints) != 4 or self.suction not in (0, 1, 2):
            raise ValueError('需要四个关节，吸盘指令为 0/1/2')
        frame = bytearray(30)
        frame[0], frame[29] = 0xCC, 0xEE
        struct.pack_into('>hhhHHB', frame, 1,
                         round(bounded(self.vx, -2, 2) * 10000),
                         round(bounded(self.vy, -2, 2) * 10000),
                         round(bounded(self.wz, -3.2, 3.2) * 10000),
                         round(bounded(self.front_mm, 0, 420)),
                         round(bounded(self.back_mm, 0, 420)), int(self.fast_lift))
        if self.arm_mode not in (0,1,2):
            raise ValueError('机械臂模式必须为 0/1/2')
        if self.arm_mode==0:
            struct.pack_into('>hhhh', frame, 12,
                             *(round(bounded(p, *limits) * 1000)
                               for p, limits in zip(self.joints, JOINT_LIMITS)))
        else:
            if len(self.cartesian)!=3:raise ValueError('坐标需要 X/Y/phi')
            x,y,phi=map(float,self.cartesian)
            if not all(math.isfinite(v) for v in (x,y,phi)) or not all(-3276.8<=v<=3276.7 for v in (x,y)):
                raise ValueError('坐标必须为有限值，X/Y 范围 -3276.8～3276.7 mm')
            struct.pack_into('>hhhh',frame,12,round(x*10),round(y*10),
                             round(math.remainder(phi,2*math.pi)*1000) if self.arm_mode==1 else 0,
                             round(bounded(self.joints[3],*JOINT_LIMITS[3])*1000))
            struct.pack_into('>H',frame,20,round(bounded(self.arm_rate,.02,10.)*1000))
        frame[28]=self.arm_mode
        frame[24] = self.suction
        return bytes(frame)


@dataclass(frozen=True)
class Status:
    front_mm: int | None
    back_mm: int | None
    joints: tuple
    arm_status: int = 0
    arm_protocol: int = 0


class StatusDecoder:
    """Bounded stream parser supporting partial frames, noise and coalesced frames."""
    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while self.buffer:
            start = self.buffer.find(b'\xcc')
            if start < 0:
                self.discarded += len(self.buffer)
                self.buffer.clear()
                break
            self.discarded += start
            del self.buffer[:start]
            if len(self.buffer) < 23:
                break
            candidate = self.buffer[:23]
            if candidate[22] != 0xEE or candidate[17:21] != bytes(4) or candidate[21] not in ARM_STATUS_TEXT:
                del self.buffer[0]
                self.discarded += 1
                continue
            front, back = struct.unpack_from('>HH', candidate, 1)
            joints = tuple(v / 1000 for v in struct.unpack_from('>hhhh', candidate, 9))
            result.append(Status(None if front == 65535 else front,
                                 None if back == 65535 else back, joints,candidate[21],candidate[8]))
            del self.buffer[:23]
        return result


def approach(current, target, max_step):
    return current + min(max_step, max(-max_step, target - current))
