import math
import struct
import time
import unittest
from unittest.mock import patch
import serial
from protocol import Command, Status, StatusDecoder, approach


def status_frame(joints=(500,1000,1500,-2000)):
    frame=bytearray(23)
    frame[0],frame[-1]=0xcc,0xee
    struct.pack_into('>HH',frame,1,30,65535)
    struct.pack_into('>hhhh',frame,9,*joints)
    return bytes(frame)


class ProtocolTests(unittest.TestCase):
    def test_wire_layout(self):
        data=Command(.2,-.3,.4,55,66,True,(.1,.2,.3,-1.234),1).encode()
        self.assertEqual(len(data),30)
        self.assertEqual(data[0],0xcc)
        self.assertEqual(data[-1],0xee)
        self.assertEqual(struct.unpack_from('>hhhHHB',data,1),(2000,-3000,4000,55,66,1))
        self.assertEqual(struct.unpack_from('>hhhh',data,12),(100,200,300,-1234))
        self.assertEqual(data[20:24],bytes(4))
        self.assertEqual(data[24],1)
        self.assertEqual(data[25:29],bytes(4))

    def test_independent_limits(self):
        data=Command(100,-100,100,-20,500,False,(-10,10,10,-10),2).encode()
        self.assertEqual(struct.unpack_from('>hhhHH',data,1),(20000,-20000,32000,0,420))
        self.assertEqual(struct.unpack_from('>hhhh',data,12),(0,5000,5000,-6280))
        self.assertEqual(struct.unpack_from('>h',Command(joints=(99,0,0,99)).encode(),18)[0],6280)
        self.assertEqual(struct.unpack_from('>h',Command(joints=(99,0,0,0)).encode(),12)[0],2500)

    def test_nonfinite_rejected(self):
        for value in (math.nan,math.inf,-math.inf):
            with self.assertRaises(ValueError): Command(vx=value).encode()
            with self.assertRaises(ValueError): Command(joints=(0,0,0,value)).encode()

    def test_all_stream_split_positions(self):
        frame=status_frame()
        for split in range(24):
            decoder=StatusDecoder()
            results=decoder.feed(frame[:split])+decoder.feed(frame[split:])
            self.assertEqual(results,[Status(30,None,(.5,1,1.5,-2))])

    def test_status_arm_result_and_capability(self):
        frame=bytearray(status_frame());frame[8]=0xc2
        for value in range(4):
            frame[21]=value
            status=StatusDecoder().feed(frame)[0]
            self.assertEqual((status.arm_status,status.arm_protocol),(value,0xc2))
        for value in (4,255):
            frame[21]=value
            self.assertEqual(StatusDecoder().feed(frame),[])

    def test_coordinate_wire_layout(self):
        for mode in (1,2):
            frame=Command(joints=(0,0,0,-1.234),arm_mode=mode,
                          cartesian=(-175.1,178.2,math.radians(30)),arm_rate=.5).encode()
            self.assertEqual(struct.unpack_from('>hhhhH',frame,12),(-1751,1782,524 if mode==1 else 0,-1234,500))
            self.assertEqual(frame[22:24],bytes(2));self.assertEqual(frame[25:28],bytes(3))
            self.assertEqual(frame[28],mode)
        with self.assertRaises(ValueError):Command(arm_mode=3).encode()
        with self.assertRaises(ValueError):Command(arm_mode=1,cartesian=(4000,0,0)).encode()
        for rate,expected in ((10,10000),(20,10000),(.001,20)):
            frame=Command(arm_mode=2,arm_rate=rate).encode()
            self.assertEqual(struct.unpack_from('>H',frame,20)[0],expected)

    def test_noise_coalescing_and_corruption(self):
        good=status_frame((204,524,0,-6280))  # 0xcc occurs inside payload.
        bad=bytearray(good); bad[18]=1
        decoder=StatusDecoder()
        result=decoder.feed(b'noise'+bad+b'garbage'+good+good[:7])
        self.assertEqual(len(result),1)
        self.assertEqual(result[0].joints,(.204,.524,0,-6.28))
        self.assertEqual(len(decoder.feed(good[7:])),1)
        self.assertLess(len(decoder.buffer),23)
        decoder.feed(b'x'*100000)
        self.assertEqual(len(decoder.buffer),0)

    def test_slew_no_overshoot(self):
        self.assertAlmostEqual(approach(0,1,.006),.006)
        self.assertAlmostEqual(approach(0,-1,.006),-.006)
        self.assertEqual(approach(.999,1,.006),1)

    def test_virtual_serial(self):
        with serial.serial_for_url('loop://',timeout=.2) as port:
            packet=Command().encode()
            self.assertEqual(port.write(packet),30)
            self.assertEqual(port.read(30),packet)


class FakePort:
    port='TEST ONLY'
    in_waiting=0
    def __init__(self): self.writes=[]; self.closed=False; self.fail=False
    def write(self,data):
        if self.fail: raise serial.SerialException('test unplug')
        self.writes.append(data)
        return len(data)
    def close(self): self.closed=True


class GuiTests(unittest.TestCase):
    def setUp(self):
        from app import RobotConsole
        self.app=RobotConsole()
        self.app.withdraw()
        self.app.update_idletasks()

    def tearDown(self): self.app.close()

    def prepare(self):
        self.app.port=FakePort()
        self.app.last_status=Status(30,40,(.2,.5,1.,-1.))
        self.app.last_rx=time.monotonic()
        self.app.sync_targets(silent=True)

    def test_numeric_slider_and_invalid_value(self):
        axis=self.app.arm[3]
        axis.text.set('-1.234'); self.assertTrue(axis.commit())
        self.assertEqual(axis.slider.get(),-1.234)
        axis._slide('2.345'); self.assertEqual(axis.text.get(),'2.345')
        axis.text.set('999'); axis.commit(); self.assertEqual(axis.value,6.28)
        axis.text.set('nan'); self.assertFalse(axis.commit()); self.assertEqual(axis.value,6.28)

    def test_arm_visual_slider_and_control_page_stay_in_sync(self):
        visual=self.app.arm_view
        visual.axes[0]._slide('.161')
        self.assertEqual(self.app.arm[0].value,.161)
        self.assertAlmostEqual(visual.target_pose.link_angles_rad[0],.161-math.radians(9))
        self.app.arm[1].text.set('.203');self.app.arm[1].commit()
        self.assertEqual(visual.axes[1].value,.203)
        self.assertEqual(len(visual.canvas.find_withtag('target-link')),3)
        self.assertEqual(len(visual.canvas.find_withtag('target-joint')),4)
        self.assertEqual(len(visual.canvas.find_withtag('joint-label')),4)
        self.assertEqual(visual.axes[0].high,2.5)
        self.assertEqual(visual.axes[1].high,5)

    def test_visual_feedback_is_fresh_only_and_sent_position_is_wire_value(self):
        self.prepare();self.app._show_status(time.monotonic())
        visual=self.app.arm_view
        self.assertIsNotNone(visual.feedback_pose)
        self.assertEqual(len(visual.canvas.find_withtag('feedback-link')),3)
        self.assertIsNone(visual.sent_pose)
        self.app.toggle_transmit();self.app.send(.02);self.app._show_status(time.monotonic())
        self.assertIsNotNone(visual.sent_pose)
        self.app.last_rx=time.monotonic()-1
        self.app._show_status(time.monotonic())
        self.assertIsNone(visual.feedback_pose)
        self.assertIn('已过期',visual.feedback_text.get())
        self.assertEqual(len(visual.canvas.find_withtag('feedback-link')),0)

    def test_visual_accepts_overlap_and_id4_does_not_move_tip(self):
        visual=self.app.arm_view
        p=visual.target_pose
        visual.axes[3].set(2)
        self.assertEqual(visual.target_pose.x_mm,p.x_mm)
        self.assertEqual(visual.target_pose.y_mm,p.y_mm)
        for axis,value in zip(visual.axes,(1.102,.143,4.481,2)):axis.set(value)
        self.assertTrue(visual._target_clear)
        self.assertIn('重合不拦截',visual.notice.get())
        self.assertEqual(visual.canvas.itemcget(visual.canvas.find_withtag('target-link')[0],'fill'),'#2469cb')

    def test_live_refresh_does_not_overwrite_number_being_typed(self):
        axis=self.app.arm_view.axes[0]
        axis.text.set('.25')
        self.app._show_status(time.monotonic())
        self.assertEqual(axis.text.get(),'.25')
        axis.commit()
        self.assertEqual(self.app.arm[0].value,.25)

    def test_connect_uses_921600_without_sending(self):
        self.app.port_map={'TEST PORT':'COM_TEST'}
        self.app.com.set('TEST PORT')
        with patch('app.serial.Serial') as constructor:
            self.app.toggle_connection()
            self.assertEqual(constructor.call_args.kwargs['baudrate'],921600)
            self.assertFalse(constructor.return_value.dtr)
            self.assertFalse(constructor.return_value.rts)
            constructor.return_value.write.assert_not_called()
            self.assertFalse(self.app.transmitting)

    def test_start_passive_and_stop_base_preserves_pose(self):
        self.prepare()
        self.assertEqual(self.app.port.writes,[])
        self.app.base[0].set(.5)
        self.app.toggle_transmit()
        self.assertTrue(self.app.transmitting)
        self.assertEqual(self.app.base[0].value,0)
        self.app.base[0].set(.3); self.app.arm[3].set(-2.)
        self.app.send(.02)
        data=self.app.port.writes[-1]
        self.assertEqual(struct.unpack_from('>h',data,1)[0],3000)
        self.assertEqual(struct.unpack_from('>h',data,18)[0],-1006)
        self.app.stop_base()
        stopped=self.app.port.writes[-1]
        self.assertEqual(stopped[1:7],bytes(6))
        self.assertEqual(stopped[12:20],data[12:20])

    def test_cartesian_control_applies_safe_solution_without_auto_start(self):
        self.prepare()
        self.app.auto_pose.set(False);self.app._ik_mode_changed()
        self.assertTrue(self.app.apply_cartesian())
        self.assertFalse(self.app.transmitting)
        self.assertEqual(self.app.port.writes,[])
        for axis,expected in zip(self.app.arm[:3],(1.043,4.440,3.397)):
            self.assertAlmostEqual(axis.value,expected)

    def test_host_ik_sends_only_joint_angle_frame(self):
        self.prepare()
        self.app.auto_pose.set(False);self.app._ik_mode_changed()
        self.assertTrue(self.app.apply_cartesian())
        self.app.toggle_transmit()
        self.app.send(100)
        frame=self.app.port.writes[-1]
        self.assertEqual(len(frame),30)
        self.assertEqual(struct.unpack_from('>hhhh',frame,12),(1043,4440,3397,-1000))
        self.assertEqual(frame[20:24],bytes(4))
        self.assertEqual(frame[25:29],bytes(4))

    def test_auto_pose_mode_xy_only_and_angle_frame(self):
        self.prepare()
        self.assertTrue(self.app.auto_pose.get())
        phi_axis=self.app.cartesian[2]
        self.assertTrue(phi_axis.scale.instate(['disabled']))
        self.assertTrue(phi_axis.spin.instate(['disabled']))
        self.app.cartesian[0].set(175);self.app.cartesian[1].set(178)
        phi_axis.text.set('ignored in automatic mode')
        self.assertTrue(self.app.apply_cartesian())
        self.assertFalse(self.app.transmitting)
        self.assertEqual(self.app.port.writes,[])
        self.assertEqual(self.app.arm[3].value,-1.)
        pose=self.app.kinematics.forward(tuple(axis.value for axis in self.app.arm[:3]),check_limits=True)
        self.assertLess(math.hypot(pose.x_mm-175,pose.y_mm-178),1.)
        self.assertAlmostEqual(phi_axis.value,round(math.degrees(math.remainder(pose.phi_rad,2*math.pi)),2))
        self.assertIn('自动姿态',self.app.ik_text.get())
        self.app.toggle_transmit();self.app.send(100)
        frame=self.app.port.writes[-1]
        self.assertEqual(len(frame),30)
        self.assertEqual(frame[20:24],bytes(4))
        self.assertEqual(frame[25:29],bytes(4))
        self.assertEqual(struct.unpack_from('>hhhh',frame,12),
                         tuple(round(axis.value*1000) for axis in self.app.arm))

    def test_fixed_pose_rejects_175_178_at_30_without_overwriting_targets(self):
        self.prepare()
        self.app.auto_pose.set(False);self.app._ik_mode_changed()
        self.assertFalse(self.app.cartesian[2].spin.instate(['disabled']))
        for axis,value in zip(self.app.cartesian,(175,178,30)):axis.set(value)
        before=tuple(axis.value for axis in self.app.arm)
        self.assertFalse(self.app.apply_cartesian())
        self.assertEqual(tuple(axis.value for axis in self.app.arm),before)
        self.assertEqual(self.app.port.writes,[])

    def coordinate_prepare(self):
        self.prepare()
        self.app.last_status=Status(30,40,(.2,.5,1.,-1.),0,0xc2)
        self.app.arm_wire_combo.current(1);self.app._wire_mode_changed()

    def test_coordinate_start_requires_application_and_new_firmware(self):
        self.coordinate_prepare();self.app.toggle_transmit()
        self.assertFalse(self.app.transmitting)
        self.assertTrue(self.app.apply_cartesian())
        self.app.last_status=Status(30,40,(.2,.5,1.,-1.))
        self.app.toggle_transmit();self.assertFalse(self.app.transmitting)
        self.assertEqual(self.app.port.writes,[])

    def test_coordinate_fixed_auto_and_staged_inputs(self):
        self.coordinate_prepare()
        self.app.auto_pose.set(False);self.app._ik_mode_changed()
        self.assertTrue(self.app.apply_cartesian())
        self.assertEqual(self.app.port.writes,[])
        self.app.toggle_transmit();self.app.send(.02)
        self.assertTrue(self.app.arm_wire_combo.instate(['disabled']))
        first=self.app.port.writes[-1]
        self.assertEqual(first[28],1)
        self.assertEqual(struct.unpack_from('>hhhhH',first,12),(8000,2000,524,-1000,300))
        self.app.cartesian[0].set(175);self.app.cartesian[1].set(178)
        self.app.auto_pose.set(True);self.app._ik_mode_changed()
        self.app.send(.02);self.assertEqual(self.app.port.writes[-1],first)
        self.assertTrue(self.app.apply_cartesian());self.app.send(.02)
        self.assertEqual(self.app.port.writes[-1][28],2)
        self.assertEqual(struct.unpack_from('>hhh',self.app.port.writes[-1],12),(1750,1780,0))
        self.app._show_status(time.monotonic())
        self.assertIsNone(self.app.arm_view.sent_pose)
        self.assertIn('最后下发坐标',self.app.arm_view.sent_text.get())
        self.assertIn('支持角度/坐标',self.app.arm_ack_text.get())
        self.assertEqual(self.app.table.item('j0','values')[2],'MCU 解算')
        self.app.toggle_transmit()
        self.assertTrue(self.app.arm_wire_combo.instate(['readonly']))
        self.app.toggle_transmit()
        self.app.allow_no_rx.set(True)
        self.app.last_rx=time.monotonic()-1
        self.app.tick()
        self.assertFalse(self.app.transmitting)
        self.assertEqual(self.app.port.writes[-1][1:7],bytes(6))

    def test_coordinate_invalid_preview_is_staged_for_mcu_validation(self):
        self.coordinate_prepare()
        self.app.auto_pose.set(False);self.app._ik_mode_changed()
        for axis,value in zip(self.app.cartesian,(175,178,30)):axis.set(value)
        self.assertTrue(self.app.apply_cartesian())
        self.assertEqual(self.app.applied_cartesian,(1,175.,178.,.524))
        self.assertIn('MCU 最终校验',self.app.ik_text.get())

    def test_overlapping_joint_target_moves_within_limits(self):
        self.prepare();self.app.toggle_transmit()
        old=self.app.current_arm[:]
        for axis,value in zip(self.app.arm,(1.102,.143,4.481,0)):axis.set(value)
        self.app.base[0].set(.3)
        self.app.send(.02)
        data=self.app.port.writes[-1]
        self.assertEqual(struct.unpack_from('>h',data,1)[0],3000)
        self.assertNotEqual(struct.unpack_from('>hhhh',data,12),tuple(round(v*1000) for v in old))
        self.assertEqual(tuple(axis.value for axis in self.app.arm),(1.102,.143,4.481,0))

    def test_cartesian_ik_accepts_overlapping_pose(self):
        self.prepare()
        pose=self.app.kinematics.forward((1.102,.143,4.481))
        for axis,value in zip(self.app.cartesian,(pose.x_mm,pose.y_mm,math.degrees(math.remainder(pose.phi_rad,2*math.pi)))):
            axis.set(value)
        self.assertTrue(self.app.apply_cartesian())
        self.assertFalse(self.app.transmitting)

    def test_small_rate_still_accumulates_sub_milliradian_steps(self):
        self.prepare();self.app.toggle_transmit();self.app.arm_rate.set('.02')
        self.app.arm[0].set(.21)
        for _ in range(10):self.app.send(.02)
        self.assertGreaterEqual(struct.unpack_from('>h',self.app.port.writes[-1],12)[0],203)
        self.app.arm_rate.set('10');self.app.arm[0].set(2.5)
        before=self.app.current_arm[0]
        self.app.send(.02)
        self.assertAlmostEqual(self.app.current_arm[0]-before,.2)

    def test_jog_release_and_disconnect(self):
        self.prepare(); self.app.toggle_transmit()
        self.app.jog((1,0,0))
        self.assertEqual(self.app.base[0].value,.2)
        self.app._release_jog()
        self.assertEqual(self.app.base[0].value,0)
        port=self.app.port
        self.app.disconnect()
        self.assertTrue(port.closed)
        self.assertEqual(port.writes[-1][1:7],bytes(6))

    def test_unplug_during_disconnect_does_not_recurse(self):
        self.prepare(); self.app.toggle_transmit()
        self.app.port.fail=True
        self.app.disconnect()
        self.assertIsNone(self.app.port)
        self.assertFalse(self.app.transmitting)

    def test_no_feedback_blocks_start(self):
        self.app.port=FakePort()
        self.app.toggle_transmit()
        self.assertFalse(self.app.transmitting)
        self.assertEqual(self.app.port.writes,[])

    def test_feedback_timeout_stops_transmit(self):
        self.prepare(); self.app.toggle_transmit()
        self.app.last_rx=time.monotonic()-1
        self.app.base[0].set(.3)
        self.app.tick()
        self.assertFalse(self.app.transmitting)
        self.assertEqual(self.app.port.writes[-1][1:7],bytes(6))


if __name__=='__main__': unittest.main(verbosity=2)
