"""Windows robot console. Connecting is passive; only Start enables transmission."""
import logging
import math
import struct
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports

from protocol import Command, JOINT_LIMITS, StatusDecoder, approach, bounded, ARM_PROTOCOL_CARTESIAN, ARM_STATUS_TEXT
from kinematics import ArmKinematics
from arm_view import ArmView

BACKGROUND = '#edf2f7'
INK = '#172b45'
BLUE = '#1368ce'


class AxisControl(ttk.Frame):
    """One validated value shared by slider and numeric entry."""
    def __init__(self, parent, name, low, high, unit, initial=0, decimals=3, changed=None, compact=False):
        super().__init__(parent)
        self.low, self.high, self.decimals = low, high, decimals
        self.changed = changed
        self.value = float(initial)
        self.text = tk.StringVar(value=f'{initial:.{decimals}f}')
        self.slider = tk.DoubleVar(value=initial)
        ttk.Label(self, text=name, width=12, style='Axis.TLabel').grid(row=0, column=0, sticky='w')
        self.scale = ttk.Scale(self, from_=low, to=high, variable=self.slider, command=self._slide)
        self.scale.grid(row=0, column=1, sticky='ew', padx=(0, 12))
        self.spin = ttk.Spinbox(self, from_=low, to=high, increment=10 ** -decimals,
                                textvariable=self.text, width=8, command=self.commit,
                                format=f'%.{decimals}f')
        self.spin.grid(row=0, column=2)
        self.spin.bind('<Return>', self.commit)
        self.spin.bind('<FocusOut>', self.commit)
        ttk.Label(self, text=unit, width=6).grid(row=0, column=3, padx=(6, 0))
        if not compact:
            ttk.Label(self, text=f'{low:g} … {high:g}', style='Muted.TLabel').grid(
                row=1, column=1, sticky='w', pady=(0, 4))
        self.columnconfigure(1, weight=1)

    def set(self, value, notify=True):
        self.value = round(bounded(value, self.low, self.high), self.decimals)
        self.slider.set(self.value)
        self.text.set(f'{self.value:.{self.decimals}f}')
        if notify and self.changed:
            self.changed()

    def _slide(self, value):
        self.set(float(value))

    def commit(self, event=None):
        try:
            self.set(float(self.text.get()))
            self.spin.configure(style='TSpinbox')
        except (ValueError, OverflowError):
            self.text.set(f'{self.value:.{self.decimals}f}')
            self.spin.configure(style='Invalid.TSpinbox')
            return False
        return True


class RobotConsole(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('ROBOCON 2027 · 整车控制台')
        width = min(1200, self.winfo_screenwidth()-80)
        height = min(920, self.winfo_screenheight()-100)
        self.geometry(f'{width}x{height}')
        self.minsize(min(1050,width), min(720,height))
        self.configure(bg=BACKGROUND)
        self.port = None
        self.transmitting = False
        self.decoder = StatusDecoder()
        self.last_status = None
        self.last_rx = 0.
        self.tx_count = self.rx_count = 0
        self.initialized = False
        self.current_arm = [0.] * 4
        self.last_tick = time.monotonic()
        self.last_packet = b''
        self.port_map = {}
        self._closing = False
        self._focus_job = None
        self.kinematics = ArmKinematics()
        self._checked_target = None
        self.applied_cartesian = None
        self._style()
        self._build()
        self.refresh_ports()
        self.bind('<Escape>', lambda event: self.stop_base())
        self.bind('<ButtonRelease-1>', self._release_jog, add='+')
        self.bind('<FocusOut>', self._focus_out, add='+')
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.after(20, self.tick)
        self.log('就绪。连接串口不会发送；先开 CH4、将 CH5 切到自动，再开始发送。')

    def _style(self):
        s = ttk.Style(self)
        s.theme_use('clam')
        s.configure('.', font=('Microsoft YaHei UI', 10), foreground=INK, background=BACKGROUND)
        s.configure('Card.TLabelframe', background='white', borderwidth=1, relief='solid')
        s.configure('Card.TLabelframe.Label', font=('Microsoft YaHei UI', 12, 'bold'))
        s.configure('TFrame', background='white')
        s.configure('TLabel', background='white')
        s.configure('Axis.TLabel', font=('Microsoft YaHei UI', 10, 'bold'))
        s.configure('Muted.TLabel', foreground='#6b7c92', font=('Microsoft YaHei UI', 9))
        s.configure('Title.TLabel', font=('Microsoft YaHei UI', 23, 'bold'), background=BACKGROUND)
        s.configure('Subtitle.TLabel', foreground='#6b7c92', background=BACKGROUND)
        s.configure('TButton', padding=(12, 7), background='#e7eef8')
        s.configure('Primary.TButton', background=BLUE, foreground='white')
        s.map('Primary.TButton', background=[('active', '#0d56af')])
        s.configure('Stop.TButton', background='#c93846', foreground='white',
                    font=('Microsoft YaHei UI', 12, 'bold'))
        s.map('Stop.TButton', background=[('active', '#a52632')])
        s.configure('TCheckbutton', background='white')
        s.configure('Invalid.TSpinbox', fieldbackground='#ffe5e5')
        s.configure('Treeview', rowheight=26, fieldbackground='white', background='white')
        s.configure('Treeview.Heading', font=('Microsoft YaHei UI', 10, 'bold'))

    def card(self, parent, title, row, column, **grid):
        frame = ttk.LabelFrame(parent, text=title, style='Card.TLabelframe', padding=14)
        frame.grid(row=row, column=column, sticky='nsew', padx=7, pady=7, **grid)
        return frame

    def _build(self):
        shell = tk.Frame(self, bg=BACKGROUND)
        shell.pack(fill='both', expand=True, padx=18, pady=14)
        ttk.Label(shell, text='ROBOCON / 整车控制台', style='Title.TLabel').pack(anchor='w')
        ttk.Label(shell, text='2027  ·  四关节机械臂  ·  全向底盘  ·  双升降  ·  YV2 + YV3',
                  style='Subtitle.TLabel').pack(anchor='w', pady=(0, 10))
        connection = ttk.Frame(shell, padding=10)
        connection.pack(fill='x')
        ttk.Label(connection, text='串口').pack(side='left', padx=(0, 8))
        self.com = ttk.Combobox(connection, state='readonly', width=31)
        self.com.pack(side='left')
        self.refresh_btn = ttk.Button(connection, text='刷新', command=self.refresh_ports)
        self.refresh_btn.pack(side='left', padx=5)
        self.connect_btn = ttk.Button(connection, text='连接', command=self.toggle_connection,
                                      style='Primary.TButton')
        self.connect_btn.pack(side='left')
        ttk.Label(connection, text='921600 / 8N1', style='Muted.TLabel').pack(side='left', padx=12)
        self.start_btn = ttk.Button(connection, text='开始发送 · 50 Hz', command=self.toggle_transmit,
                                    state='disabled', style='Primary.TButton')
        self.start_btn.pack(side='right')
        self.connection_text = tk.StringVar(value='● 未连接')
        ttk.Label(connection, textvariable=self.connection_text).pack(side='right', padx=15)
        note = tk.Label(shell, anchor='w', bg='#fff1d6', fg='#77521d', padx=12, pady=7,
                        text='接线：DAP TX → PA10   DAP RX ← PA9   GND 共地   |   遥控器 CH4 高使能、CH5 高自动')
        note.pack(fill='x', pady=(8, 4))
        # Keep stop controls pinned; scroll the cards on small/high-DPI screens.
        body_host = ttk.Frame(shell)
        self.notebook=ttk.Notebook(body_host)
        self.notebook.pack(fill='both',expand=True)
        self.visual_tab=ttk.Frame(self.notebook)
        self.controls_tab=ttk.Frame(self.notebook)
        self.notebook.add(self.visual_tab,text='机械臂画面与末端坐标')
        self.notebook.add(self.controls_tab,text='整车控制与坐标逆解')
        self.visual_canvas=tk.Canvas(self.visual_tab,bg='white',highlightthickness=0)
        visual_scroll=ttk.Scrollbar(self.visual_tab,orient='vertical',command=self.visual_canvas.yview)
        self.visual_canvas.configure(yscrollcommand=visual_scroll.set)
        visual_scroll.pack(side='right',fill='y');self.visual_canvas.pack(side='left',fill='both',expand=True)
        self.arm_view=ArmView(self.visual_canvas,self.kinematics,JOINT_LIMITS,AxisControl,self._visual_arm_changed)
        visual_item=self.visual_canvas.create_window((0,0),window=self.arm_view,anchor='nw')
        self.arm_view.bind('<Configure>',lambda event:self.visual_canvas.configure(scrollregion=self.visual_canvas.bbox('all')))
        self.visual_canvas.bind('<Configure>',lambda event:self.visual_canvas.itemconfigure(visual_item,width=event.width))
        self.body_canvas = tk.Canvas(self.controls_tab, bg=BACKGROUND, highlightthickness=0)
        scroll = ttk.Scrollbar(self.controls_tab, orient='vertical', command=self.body_canvas.yview)
        self.body_canvas.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.body_canvas.pack(side='left', fill='both', expand=True)
        body = tk.Frame(self.body_canvas, bg=BACKGROUND)
        body_item = self.body_canvas.create_window((0,0), window=body, anchor='nw')
        body.bind('<Configure>', lambda event: self.body_canvas.configure(
            scrollregion=self.body_canvas.bbox('all')))
        self.body_canvas.bind('<Configure>', lambda event: self.body_canvas.itemconfigure(
            body_item, width=event.width))
        self.bind('<MouseWheel>', self._scroll_cards, add='+')
        body.columnconfigure(0, weight=1, uniform='cards')
        body.columnconfigure(1, weight=1, uniform='cards')
        drive = self.card(body, '01 / 全向底盘', 0, 0)
        self.base = [AxisControl(drive, n, lo, hi, unit, decimals=2)
                     for n, lo, hi, unit in [('纵向 vx',-2,2,'m/s'),('横向 vy',-2,2,'m/s'),
                                             ('旋转 wz',-3.2,3.2,'rad/s')]]
        for axis in self.base:
            axis.pack(fill='x', pady=2)
        pad = ttk.Frame(drive)
        pad.pack(fill='x', pady=(6, 0))
        self.jog_active = False
        jogs = [('前进',0,1,(1,0,0)),('左移',1,0,(0,1,0)),('后退',1,1,(-1,0,0)),
                ('右移',1,2,(0,-1,0)),('左旋',0,0,(0,0,1)),('右旋',0,2,(0,0,-1))]
        for title, row, col, direction in jogs:
            b = ttk.Button(pad, text=title)
            b.grid(row=row, column=col, padx=3, pady=3, sticky='ew')
            b.bind('<ButtonPress-1>', lambda e, d=direction: self.jog(d))
            b.bind('<ButtonRelease-1>', self._release_jog)
            b.bind('<Leave>', self._release_jog)
        for col in range(3):
            pad.columnconfigure(col, weight=1)
        settings = ttk.Frame(drive)
        settings.pack(fill='x', pady=(6, 0))
        ttk.Label(settings, text='按住行驶；松开停车', style='Muted.TLabel').pack(side='left')
        ttk.Label(settings, text='速度').pack(side='left', padx=(10, 4))
        self.jog_speed = tk.StringVar(value='0.20')
        ttk.Spinbox(settings, from_=0.05, to=1., increment=.05, textvariable=self.jog_speed,
                    width=6).pack(side='left')
        ttk.Label(drive, text='方向按协议坐标，首次低速核对车体实际方向。',
                  style='Muted.TLabel').pack(anchor='w', pady=(6, 0))

        arm = self.card(body, '02 / 四关节机械臂', 0, 1)
        self.arm = []
        for i, limits in enumerate(JOINT_LIMITS):
            axis = AxisControl(arm, f'关节 ID{i+1}', *limits, 'rad',changed=self._arm_target_changed)
            axis.pack(fill='x', pady=2)
            self.arm.append(axis)
        arm_bottom = ttk.Frame(arm)
        arm_bottom.pack(fill='x', pady=(6,0))
        self.sync_btn = ttk.Button(arm_bottom, text='用当前反馈填充目标', command=self.sync_targets)
        self.sync_btn.pack(side='left')
        ttk.Label(arm_bottom, text='限速 rad/s').pack(side='left', padx=(12, 4))
        self.arm_rate = tk.StringVar(value='0.30')
        ttk.Spinbox(arm_bottom, from_=0.02, to=10., increment=.05,
                    textvariable=self.arm_rate, width=6).pack(side='left')
        ttk.Label(arm, text='滑条与数值框联动；输入后按 Enter 或移开焦点生效。\n机械臂按限速平滑跟随，ID4 支持负角度。',
                  style='Muted.TLabel').pack(anchor='w', pady=(8,0))

        cart = self.card(body, '05 / 坐标逆解与电机限幅', 2, 0, columnspan=2)
        wire_row=ttk.Frame(cart);wire_row.pack(fill='x',pady=(0,6))
        ttk.Label(wire_row,text='串口发送').pack(side='left',padx=(0,8))
        self.arm_wire_mode=tk.StringVar(value='角度帧（上位机解算）')
        self.arm_wire_combo=ttk.Combobox(wire_row,state='readonly',width=30,textvariable=self.arm_wire_mode,
                         values=('角度帧（上位机解算）','坐标帧（单片机解算）'))
        self.arm_wire_combo.pack(side='left')
        self.arm_wire_combo.bind('<<ComboboxSelected>>',self._wire_mode_changed)
        ttk.Label(wire_row,text='坐标帧需新固件；发送前点击下方按钮应用坐标。',style='Muted.TLabel').pack(side='left',padx=12)
        mode_row=ttk.Frame(cart); mode_row.pack(fill='x',pady=(0,6))
        self.auto_pose=tk.BooleanVar(value=True)
        ttk.Radiobutton(mode_row,text='自动选择姿态（只输入 X、Y）',variable=self.auto_pose,
                        value=True,command=self._ik_mode_changed).pack(side='left')
        ttk.Radiobutton(mode_row,text='固定连杆3姿态',variable=self.auto_pose,
                        value=False,command=self._ik_mode_changed).pack(side='left',padx=16)
        self.cartesian = [AxisControl(cart,name,lo,hi,unit,initial=value,decimals=dec)
            for name,lo,hi,unit,value,dec in [('末端 X',-1050,1050,'mm',800,1),
                ('末端 Y',-1050,1050,'mm',200,1),('连杆3姿态',-180,180,'deg',30,2)]]
        for axis in self.cartesian: axis.pack(fill='x')
        settings=ttk.Frame(cart); settings.pack(fill='x',pady=6)
        ttk.Button(settings,text='应用坐标 / 求逆解预览',command=self.apply_cartesian,
                   style='Primary.TButton').pack(side='left')
        ttk.Label(settings,text='逆解条件：几何可达 + 电机限幅').pack(side='left',padx=12)
        self.ik_text=tk.StringVar(value='连杆相交、重合及路径重合不参与逆解限制。')
        ttk.Label(cart,textvariable=self.ik_text,wraplength=950).pack(anchor='w')
        self._ik_mode_changed()
        self.arm_ack_text=tk.StringVar(value='MCU 机械臂状态：尚无回传')
        ttk.Label(cart,textvariable=self.arm_ack_text,wraplength=950).pack(anchor='w',pady=(4,0))

        lift = self.card(body, '03 / 升降与吸盘', 1, 0)
        self.lift = [AxisControl(lift, label, 0, 420, 'mm', initial=30, decimals=0)
                     for label in ['前升降','后升降']]
        for axis in self.lift:
            axis.pack(fill='x')
        self.fast_lift = tk.BooleanVar(value=False)
        ttk.Checkbutton(lift, text='升降快速模式', variable=self.fast_lift).pack(anchor='w')
        suction_row = ttk.Frame(lift)
        suction_row.pack(fill='x', pady=(8,0))
        ttk.Label(suction_row, text='YV2 + YV3').pack(side='left', padx=(0,10))
        self.suction = tk.IntVar(value=0)
        for label, value in [('保持',0),('同时吸气',1),('同时释放',2)]:
            ttk.Radiobutton(suction_row, text=label, variable=self.suction,
                            value=value).pack(side='left', padx=3)

        feedback = self.card(body, '04 / 实时反馈', 1, 1)
        self.table = ttk.Treeview(feedback, columns=('actual','target','sent'), show='tree headings', height=6)
        for name, label, width in [('#0','执行器',100),('actual','回传',95),('target','目标',95),('sent','下发',95)]:
            self.table.heading(name,text=label)
            self.table.column(name,width=width,anchor='center',stretch=True)
        self.table.pack(fill='both', expand=True)
        for key,label in [('front','前升降 mm'),('back','后升降 mm')]+[(f'j{i}',f'ID{i+1} rad') for i in range(4)]:
            self.table.insert('', 'end', iid=key, text=label, values=('—','—','—'))
        self.feedback_text = tk.StringVar(value='尚未收到回传；连接成功不代表控制已生效')
        ttk.Label(feedback, textvariable=self.feedback_text, style='Muted.TLabel').pack(anchor='w', pady=(5,0))

        bottom = ttk.Frame(shell, padding=10)
        bottom.pack(side='bottom', fill='x', pady=(6, 0))
        ttk.Button(bottom, text='底盘停车 / Esc', style='Stop.TButton',
                   command=self.stop_base).pack(side='left')
        ttk.Label(bottom, text='只停止底盘；全车失能请关遥控器 CH4。',
                  style='Muted.TLabel').pack(side='left', padx=12)
        self.allow_no_rx = tk.BooleanVar(value=False)
        ttk.Checkbutton(bottom, text='仅发送接线（无回传）', variable=self.allow_no_rx,
                        command=self._open_loop_change).pack(side='right')
        self.counters = tk.StringVar(value='TX 0  /  RX 0')
        ttk.Label(bottom, textvariable=self.counters).pack(side='right', padx=15)
        self.logbox = tk.Text(shell, height=3, bg='#182b43', fg='#d7e6f5', relief='flat',
                              font=('Microsoft YaHei UI',9), padx=10, pady=5, state='disabled')
        self.logbox.pack(side='bottom', fill='x', pady=(8,0))
        body_host.pack(fill='both', expand=True)
        self._show_arm_view(time.monotonic())

    def _scroll_cards(self, event):
        if event.widget.winfo_class() not in ('TSpinbox','TScale'):
            canvas=self.visual_canvas if self.notebook.select()==str(self.visual_tab) else self.body_canvas
            canvas.yview_scroll(-int(event.delta/120),'units')

    def _visual_arm_changed(self,index,value):
        self.arm[index].set(value)

    def _arm_target_changed(self):
        self._checked_target=None
        if hasattr(self,'arm') and len(self.arm)==4:
            self._show_arm_view(time.monotonic())

    def _show_arm_view(self,now):
        fresh=self.port is not None and self.last_status is not None and now-self.last_rx<.6
        feedback=self.last_status.joints if fresh else None
        state='回传有效' if fresh else ('回传已过期' if self.last_status else '尚无回传')
        sent=None
        if self.last_packet and self.last_packet[28]==0:
            sent=tuple(value*.001 for value in struct.unpack_from('>hhhh',self.last_packet,12))
        self.arm_view.update_pose(tuple(axis.value for axis in self.arm),feedback,sent,state)
        if self.last_packet and self.last_packet[28] in (1,2):
            x,y,phi,q4=struct.unpack_from('>hhhh',self.last_packet,12)
            orientation='自动姿态' if self.last_packet[28]==2 else f'φ={math.degrees(phi*.001):.2f}°'
            self.arm_view.sent_text.set(f'最后下发坐标：X={x*.1:.1f} mm  Y={y*.1:.1f} mm  {orientation}；关节以 MCU 回传为准')

    def log(self, text):
        logging.info(text)
        self.logbox.configure(state='normal')
        self.logbox.insert('end', time.strftime('%H:%M:%S')+'  '+text+'\n')
        if int(self.logbox.index('end-1c').split('.')[0]) > 150:
            self.logbox.delete('1.0','30.0')
        self.logbox.see('end')
        self.logbox.configure(state='disabled')

    def refresh_ports(self):
        if self.port:
            return
        previous = self.com.get()
        ports = sorted(list_ports.comports(), key=lambda p: (not bool(p.vid), p.device))
        self.port_map = {f'{p.device} · {p.description}':p.device for p in ports}
        self.com['values'] = list(self.port_map)
        if previous in self.port_map:
            self.com.set(previous)
        elif ports:
            self.com.current(0)
        else:
            self.com.set('')

    def toggle_connection(self):
        if self.port:
            self.disconnect()
            return
        device = self.port_map.get(self.com.get())
        if not device:
            messagebox.showinfo('选择串口', '请先选择 DAP 对应的串口。')
            return
        opened = serial.Serial(port=None, baudrate=921600, timeout=0, write_timeout=.04,
                               bytesize=8, parity='N', stopbits=1)
        opened.dtr = False
        opened.rts = False
        try:
            opened.port = device
            opened.open()
            opened.reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            opened.close()
            messagebox.showerror('连接失败', str(exc))
            return
        self.port = opened
        self.decoder = StatusDecoder()
        self.last_rx = 0.
        self.last_status = None
        self.initialized = False
        self.stop_base(send=False)
        self.com.configure(state='disabled')
        self.refresh_btn.configure(state='disabled')
        self.connect_btn.configure(text='断开')
        self.start_btn.configure(state='normal')
        self.connection_text.set(f'● {device} 已连接 / 未发送')
        self.log(f'{device} 已连接，当前只接收。')

    def disconnect(self, failed=False):
        if not self.port:
            return
        opened = self.port
        if self.transmitting and not failed:
            self.stop_base()
        self.transmitting = False
        self.arm_wire_combo.configure(state='readonly')
        try:
            opened.close()
        except (OSError, serial.SerialException):
            pass
        self.port = None
        self.jog_active = False
        self.initialized = False
        self.com.configure(state='readonly')
        self.refresh_btn.configure(state='normal')
        self.connect_btn.configure(text='连接')
        self.start_btn.configure(text='开始发送 · 50 Hz', state='disabled')
        self.connection_text.set('● 未连接')
        self.log('串口已断开。')

    def sync_targets(self, silent=False):
        if self.transmitting:
            if not silent:
                self.log('先停止发送，再将当前反馈填入目标。')
            return False
        if not self.last_status or time.monotonic()-self.last_rx > .6:
            if not silent:
                self.log('无新鲜回传，无法读取当前位置。核对 DAP RX←PA9、CH4、CH5。')
            return False
        status = self.last_status
        for axis,value in zip(self.arm,status.joints):
            axis.set(value)
        for axis,value in zip(self.lift,[status.front_mm,status.back_mm]):
            if value is not None:
                axis.set(value)
        self.current_arm = [axis.value for axis in self.arm]
        self._checked_target = None
        self.initialized = True
        if not silent:
            self.log('已用当前回传填充机械臂与升降目标。')
        return True

    def toggle_transmit(self):
        if not self.port:
            return
        if self.transmitting:
            self.stop_base()
            if not self.port:
                return
            self.transmitting = False
            self.arm_wire_combo.configure(state='readonly')
            self.start_btn.configure(text='开始发送 · 50 Hz')
            self.connection_text.set(f'● {self.port.port} 已连接 / 未发送')
            self.log('已发送底盘零速并停止发送；机械臂保持最后下发位置。')
            return
        # Commit numeric entries before constructing commands.
        if not all(axis.commit() for axis in self.base+self.arm+self.lift):
            self.log('请修正输入数值后再开始。')
            return
        fresh = self.last_status is not None and time.monotonic()-self.last_rx < .6
        if self.coordinate_mode():
            if self.applied_cartesian is None:
                self.log('请先点击“应用坐标 / 求逆解预览”，再开始发送坐标帧。')
                return
            if not fresh or self.last_status.arm_protocol!=ARM_PROTOCOL_CARTESIAN:
                self.log('坐标帧需要新固件的有效双向回传（协议标识 C2）；请更新固件并检查 RX 接线。')
                return
        if not fresh and not self.allow_no_rx.get():
            self.log('尚无有效回传：请检查接线与 CH4/CH5；仅接 TX 时勾选“仅发送接线”。')
            return
        if fresh:
            self.current_arm = [bounded(value,*limit) for value,limit in zip(self.last_status.joints,JOINT_LIMITS)]
        else:
            if not messagebox.askokcancel('无回传模式',
                    '无法读取当前位置，首帧会采用界面上的升降和关节目标。\n请核对各目标与机器人当前位置，再开始发送。'):
                return
            self.current_arm = [axis.value for axis in self.arm]
        self.stop_base(send=False)
        try:
            self.kinematics.forward(tuple(a.value for a in self.arm[:3]),check_limits=True)
        except ValueError as exc:
            self.log(f'不能开始机械臂控制：{exc}')
            return
        self.transmitting = True
        self.arm_wire_combo.configure(state='disabled')
        self.last_tick = time.monotonic()
        self.start_btn.configure(text='停止发送')
        self.connection_text.set(f'● {self.port.port} 正在发送')
        self.log('开始 50 Hz 控制。底盘初始零速，机械臂目标按设定限速跟随。')

    def _open_loop_change(self):
        if self.transmitting:
            self.toggle_transmit()
        self.log('仅发送模式需手动核对初始目标；无回传时不能判断机器人是否执行。'
                 if self.allow_no_rx.get() else '已启用回传超时检测。')

    def command(self, dt=0.):
        try:
            rate = bounded(self.arm_rate.get(), .02, 10.)
        except ValueError:
            rate = .3
        if self.coordinate_mode():
            if self.applied_cartesian is None:
                raise ValueError('尚未应用坐标目标')
            mode,x,y,phi=self.applied_cartesian
            if self.last_status and time.monotonic()-self.last_rx<.6:
                self.current_arm=list(self.last_status.joints)
            return Command(*(axis.value for axis in self.base),
                           *(axis.value for axis in self.lift),self.fast_lift.get(),
                           tuple(self.current_arm[:3])+ (self.arm[3].value,),self.suction.get(),
                           arm_mode=mode,cartesian=(x,y,phi),arm_rate=rate)
        target=tuple(a.value for a in self.arm[:3])
        self.kinematics.forward(target,check_limits=True)
        delta=max(abs(a-b) for a,b in zip(self.current_arm[:3],target))
        ratio=min(1.,rate*dt/delta) if delta else 1.
        proposed=[a+(b-a)*ratio for a,b in zip(self.current_arm[:3],target)]
        self.current_arm=proposed+[approach(self.current_arm[3],self.arm[3].value,rate*dt)]
        return Command(*(axis.value for axis in self.base),
                       *(axis.value for axis in self.lift), self.fast_lift.get(),
                       tuple(self.current_arm), self.suction.get())

    def send(self, dt=0.):
        if not self.port:
            return False
        try:
            packet = self.command(dt).encode()
        except ValueError as exc:
            self.log(f'机械臂目标已拒绝：{exc}')
            self.ik_text.set(str(exc))
            for axis,value in zip(self.arm,self.current_arm): axis.set(value)
            for axis in self.base: axis.set(0)
            self._checked_target=None
            # Keep the previous arm target and explicitly stop the chassis.
            packet=Command(0,0,0,*(a.value for a in self.lift),self.fast_lift.get(),
                           tuple(self.current_arm),self.suction.get()).encode()
        try:
            if self.port.write(packet) != len(packet):
                raise serial.SerialException('串口未完整写出控制帧')
        except (serial.SerialException, OSError) as exc:
            self.log(f'发送失败：{exc}')
            self.disconnect(failed=True)
            return False
        self.last_packet = packet
        self.tx_count += 1
        return True

    def _ik_mode_changed(self):
        automatic=self.auto_pose.get()
        for widget in (self.cartesian[2].scale,self.cartesian[2].spin):
            widget.state(['disabled'] if automatic else ['!disabled'])
        self.ik_text.set('自动模式：仅输入 X、Y，优先选择接近当前关节角度的限幅内解。'
                         if automatic else '固定模式：同时满足 X、Y 和连杆3姿态；无合法解则拒绝。')

    def coordinate_mode(self):
        return self.arm_wire_mode.get()=='坐标帧（单片机解算）'

    def _wire_mode_changed(self,event=None):
        self.applied_cartesian=None
        self.ik_text.set('坐标帧由 MCU 解算；请应用坐标目标。ID1～3 滑条只作预览，ID4 仍为旋转目标。'
                         if self.coordinate_mode() else '角度帧：可直接控制滑条，也可在上位机解算后发送关节角度。')

    def apply_cartesian(self):
        automatic=self.auto_pose.get()
        if not all(axis.commit() for axis in self.cartesian[:2 if automatic else 3]): return False
        # Use current sent position during motion; otherwise prefer fresh feedback.
        seed=tuple(self.current_arm[:3])
        if not self.transmitting and self.last_status and time.monotonic()-self.last_rx<.6:
            seed=tuple(self.last_status.joints[:3])
        elif not self.transmitting:
            self.ik_text.set('请先连接并接收当前位置，再应用坐标目标。')
            return False
        coordinate=self.coordinate_mode()
        x,y,phi=(axis.value for axis in self.cartesian)
        phi=math.radians(phi)
        if coordinate:
            # Stage exactly the quantized payload; editing inputs alone never changes it.
            x,y=round(x*10)/10,round(y*10)/10
            phi=round(math.remainder(phi,2*math.pi)*1000)/1000 if not automatic else 0.
            self.applied_cartesian=(2 if automatic else 1,x,y,phi)
        try:
            solution=(self.kinematics.solve_position(x,y,seed) if automatic else
                      self.kinematics.solve(x,y,phi,seed))
            rounded=tuple(round(v,3) for v in solution)
            pose=self.kinematics.forward(rounded,check_limits=True)
        except ValueError as exc:
            if coordinate:
                self.ik_text.set(f'已应用坐标 X={x:.1f}、Y={y:.1f} mm；上位机预览无解：{exc}。MCU 最终校验，失败保留原目标。')
                self.log(self.ik_text.get());return True
            self.ik_text.set(f'逆解拒绝：{exc}')
            self.log(self.ik_text.get());return False
        for axis,value in zip(self.arm[:3],rounded): axis.set(value)
        if automatic:
            self.cartesian[2].set(math.degrees(math.remainder(pose.phi_rad,2*math.pi)))
        self._checked_target=None
        self.ik_text.set(('自动姿态' if automatic else '固定姿态')+
                         f' φ={math.degrees(math.remainder(pose.phi_rad,2*math.pi)):.2f}°；'+
                         '已设置关节目标 rad：'+', '.join(f'{v:.3f}' for v in rounded)+
                         f'；1 mrad 量化后 X={pose.x_mm:.2f}，Y={pose.y_mm:.2f} mm')
        if coordinate:
            self.ik_text.set(f'已应用坐标 X={x:.1f}、Y={y:.1f} mm；'+self.ik_text.get()+'；滑条仅预览，MCU 最终解算。')
        self.log(self.ik_text.get())
        return True

    def stop_base(self, send=True):
        self.jog_active = False
        for axis in self.base:
            axis.set(0)
        if send and self.transmitting:
            self.send()

    def jog(self, direction):
        if not self.transmitting:
            self.log('先连接并开始发送，再按住行驶按钮。')
            return
        try:
            speed = bounded(self.jog_speed.get(), .05, 1.)
        except ValueError:
            speed = .2
        self.jog_active = True
        for axis,component in zip(self.base,direction):
            axis.set(component*speed)

    def _release_jog(self, event=None):
        if self.jog_active:
            self.stop_base()

    def _focus_out(self, event=None):
        if self._focus_job:
            self.after_cancel(self._focus_job)
        self._focus_job = self.after(60,self._check_focus)

    def _check_focus(self):
        self._focus_job = None
        if self.focus_displayof() is None:
            self.stop_base()

    def tick(self):
        if self._closing:
            return
        now = time.monotonic()
        dt = min(.05,max(0,now-self.last_tick))
        self.last_tick = now
        if self.port:
            try:
                waiting = self.port.in_waiting
                if waiting:
                    for status in self.decoder.feed(self.port.read(min(waiting,8192))):
                        self.last_status, self.last_rx = status, now
                        self.rx_count += 1
                    if not self.initialized and not self.transmitting:
                        self.sync_targets(silent=True)
            except (serial.SerialException,OSError) as exc:
                self.log(f'接收失败：{exc}')
                self.disconnect(failed=True)
        if self.transmitting:
            if (self.coordinate_mode() or not self.allow_no_rx.get()) and now-self.last_rx > .6:
                self.toggle_transmit()
                self.log('回传超过 600 ms 未更新，已停止控制发送并尝试发送底盘零速。')
            else:
                self.send(dt)
        self._show_status(now)
        self.after(20,self.tick)

    def _show_status(self, now):
        self._show_arm_view(now)
        fresh = self.port is not None and self.last_rx and now-self.last_rx < .6
        status = self.last_status if fresh else None
        age = f'{(now-self.last_rx)*1000:.0f} ms' if self.port and self.last_rx else '—'
        self.feedback_text.set(f'回传帧龄 {age}  ·  帧有效不代表电机在线或已执行')
        self.arm_ack_text.set(('MCU 机械臂状态：'+ARM_STATUS_TEXT[status.arm_status]+
                              ('；支持角度/坐标帧' if status.arm_protocol==ARM_PROTOCOL_CARTESIAN else '；旧固件仅支持角度帧'))
                             if status else 'MCU 机械臂状态：无有效回传')
        for i,axis in enumerate(self.lift):
            actual = ([status.front_mm,status.back_mm][i] if status else None)
            self.table.item(['front','back'][i],values=(str(actual) if actual is not None else '—',
                            f'{axis.value:.0f}',f'{axis.value:.0f}' if self.transmitting else '—'))
        for i,axis in enumerate(self.arm):
            sent=f'{self.current_arm[i]:.3f}' if self.transmitting else '—'
            if self.transmitting and self.last_packet and self.last_packet[28] in (1,2):
                sent='MCU 解算' if i<3 else f'{struct.unpack_from(">h",self.last_packet,18)[0]*.001:.3f}'
            self.table.item(f'j{i}',values=(f'{status.joints[i]:.3f}' if status else '—',
                            f'{axis.value:.3f}',sent))
        self.counters.set(f'TX {self.tx_count}  /  RX {self.rx_count}')

    def close(self):
        self.disconnect()
        self._closing = True
        self.destroy()


def main():
    logdir = Path(os.environ.get('LOCALAPPDATA',str(Path.home()))) / 'Robocon2027'
    logdir.mkdir(parents=True,exist_ok=True)
    handler = RotatingFileHandler(logdir/'console.log',maxBytes=1_000_000,backupCount=2,encoding='utf-8')
    logging.basicConfig(level=logging.INFO,handlers=[handler],format='%(asctime)s %(message)s')
    app = RobotConsole()
    app.mainloop()


if __name__ == '__main__':
    main()
