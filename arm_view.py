"""Live Tk arm diagram and synchronized joint controls; no serial I/O."""
import math
import tkinter as tk
from tkinter import ttk



class ArmView(ttk.Frame):
    def __init__(self, parent, kinematics, limits, axis_factory, on_change):
        super().__init__(parent, padding=12)
        self.kinematics = kinematics
        self.target_pose = None
        self.feedback_pose = None
        self.sent_pose = None
        self._key = None
        self.target_text = tk.StringVar()
        self.feedback_text = tk.StringVar()
        self.sent_text = tk.StringVar()
        self.notice = tk.StringVar(value='图中位置终点为 ID4 旋转中心')
        ttk.Label(self,text='机械臂平面画面',font=('Microsoft YaHei UI',14,'bold')).pack(anchor='w')
        ttk.Label(self,textvariable=self.target_text).pack(anchor='w',pady=(5,0))
        ttk.Label(self,textvariable=self.feedback_text).pack(anchor='w')
        ttk.Label(self,textvariable=self.sent_text,style='Muted.TLabel').pack(anchor='w')
        self.canvas=tk.Canvas(self,height=215,background='white',highlightthickness=0)
        self.canvas.pack(fill='x',pady=(5,0))
        self.canvas.bind('<Configure>',lambda event:self.draw())
        legend=ttk.Frame(self);legend.pack(fill='x',pady=(0,4))
        ttk.Label(legend,text='━━ 目标预览',foreground='#2469cb').pack(side='left',padx=(0,18))
        ttk.Label(legend,text='┄┄ 串口反馈',foreground='#12816a').pack(side='left',padx=(0,18))
        ttk.Label(legend,text='L1=425 · L2=425 · L3=200 mm',style='Muted.TLabel').pack(side='left')
        self.axes=[]
        for i,(lo,hi) in enumerate(limits):
            axis=axis_factory(self,f'ID{i+1}',lo,hi,'rad',compact=True,
                              changed=lambda i=i:on_change(i,self.axes[i].value))
            axis.pack(fill='x',pady=1)
            self.axes.append(axis)
        self.notice_label=ttk.Label(self,textvariable=self.notice,wraplength=900)
        self.notice_label.pack(anchor='w',pady=(5,0))

    @staticmethod
    def pose_text(prefix,pose):
        if pose is None:return prefix+'：—'
        return f'{prefix}：X={pose.x_mm:.2f} mm    Y={pose.y_mm:.2f} mm    φ={math.degrees(pose.phi_rad):.2f}°'

    def update_pose(self,target,feedback=None,sent=None,feedback_state='无回传'):
        target=tuple(target)
        feedback=None if feedback is None else tuple(feedback)
        sent=None if sent is None else tuple(sent)
        key=(target,feedback,sent,feedback_state)
        if key==self._key:return
        self._key=key
        for axis,value in zip(self.axes,target):
            if axis.value!=round(value,axis.decimals):axis.set(value,notify=False)
        self.target_pose=self.kinematics.forward(target[:3])
        self.feedback_pose=None if feedback is None else self.kinematics.forward(feedback[:3])
        self.sent_pose=None if sent is None else self.kinematics.forward(sent[:3])
        self.target_text.set(self.pose_text('目标末端',self.target_pose))
        self.feedback_text.set(self.pose_text('反馈末端',self.feedback_pose)+f'  ·  {feedback_state}')
        self.sent_text.set(self.pose_text('最后下发末端',self.sent_pose))
        try:
            self.kinematics.forward(target[:3],check_limits=True)
            self._target_clear=True
            self.notice.set('当前仅按电机限幅控制，连杆重合不拦截。ID4 旋转不改变图中末端中心位置。')
            self.notice_label.configure(foreground='#527064')
        except ValueError:
            self._target_clear=False
            self.notice.set('目标超出电机限位，无法下发。')
            self.notice_label.configure(foreground='#b42335')
        self.draw()

    def draw(self):
        c=self.canvas;c.delete('all')
        if self.target_pose is None:return
        width=max(320,c.winfo_width());height=max(220,c.winfo_height())
        points=list(self.target_pose.points_mm)
        if self.feedback_pose:points+=list(self.feedback_pose.points_mm)
        xmin=min(p[0] for p in points)-70;xmax=max(p[0] for p in points)+70
        ymin=min(p[1] for p in points)-70;ymax=max(p[1] for p in points)+70
        if xmax-xmin<350:
            mid=(xmin+xmax)/2;xmin,xmax=mid-175,mid+175
        if ymax-ymin<300:
            mid=(ymin+ymax)/2;ymin,ymax=mid-150,mid+150
        scale=min((width-105)/(xmax-xmin),(height-65)/(ymax-ymin))
        left=70+(width-105-(xmax-xmin)*scale)/2
        top=24+(height-65-(ymax-ymin)*scale)/2
        X=lambda value:left+(value-xmin)*scale
        Y=lambda value:top+(ymax-value)*scale
        # Same metric scale on both axes, responsive bounds include both poses.
        self.transform=(X,Y,scale)
        step=100 if max(xmax-xmin,ymax-ymin)<1000 else 250
        for x in range(math.ceil(xmin/step)*step,math.floor(xmax/step)*step+1,step):
            c.create_line(X(x),Y(ymin),X(x),Y(ymax),fill='#e8edf3',tags='grid')
            c.create_text(X(x),height-13,text=str(x),fill='#718096',font=('Microsoft YaHei UI',9),tags='tick')
        for y in range(math.ceil(ymin/step)*step,math.floor(ymax/step)*step+1,step):
            c.create_line(X(xmin),Y(y),X(xmax),Y(y),fill='#e8edf3',tags='grid')
            c.create_text(X(xmin)-10,Y(y),text=str(y),anchor='e',fill='#718096',font=('Microsoft YaHei UI',9),tags='tick')
        c.create_line(X(xmin),Y(0),X(xmax),Y(0),fill='#8c98a8',arrow='last',tags='axis')
        c.create_line(X(0),Y(ymin),X(0),Y(ymax),fill='#8c98a8',arrow='last',tags='axis')
        c.create_text(width-5,max(16,min(height-34,Y(0)-12)),text='X → mm',anchor='e',fill='#314154',tags='axis-label')
        c.create_text(min(width-75,max(55,X(0)+10)),12,text='Y ↑ mm',anchor='w',fill='#314154',tags='axis-label')
        if self.feedback_pose:
            for i in range(3):
                a,b=self.feedback_pose.points_mm[i:i+2]
                c.create_line(X(a[0]),Y(a[1]),X(b[0]),Y(b[1]),fill='#12816a',width=3,
                              dash=(6,4),tags=('feedback-link',f'feedback-link-{i+1}'))
            x,y=self.feedback_pose.points_mm[-1]
            c.create_oval(X(x)-7,Y(y)-7,X(x)+7,Y(y)+7,outline='#12816a',width=2,tags='feedback-tip')
        color='#2469cb' if self._target_clear else '#c93846'
        for i in range(3):
            a,b=self.target_pose.points_mm[i:i+2]
            c.create_line(X(a[0]),Y(a[1]),X(b[0]),Y(b[1]),fill=color,width=6,capstyle='round',
                          tags=('target-link',f'target-link-{i+1}'))
        labels=[]
        for i,(x,y) in enumerate(self.target_pose.points_mm):
            px,py=X(x),Y(y)
            c.create_oval(px-5,py-5,px+5,py+5,fill='white' if i<3 else color,outline=color,width=2,
                          tags=('target-joint',f'joint-{i+1}'))
            lx=max(30,min(width-30,px));ly=py
            for delta in ([24,-18,43,-38] if i%2 else [-18,24,-38,43]):
                ly=max(19,min(height-34,py+delta))
                if not any(abs(lx-a)<38 and abs(ly-b)<19 for a,b in labels):break
            labels.append((lx,ly))
            c.create_text(lx,ly,text=f'ID{i+1}',fill='#23344a',font=('Microsoft YaHei UI',10,'bold'),tags='joint-label')
