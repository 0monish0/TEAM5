Purpose:
 - Use YOLO (ultralytics) + simple color/circle post-processing to detect a circular target.
 - Compute offset (dx, dy) from frame center and smoothed target area.
 - Use a PID controller (P or PI or PID) to convert offsets into velocity commands.
 - Map offsets into one of six allowed robot motions:
     FORWARD, BACKWARD, LEFT, RIGHT, ROT_CW, ROT_CCW, STOP
 - Send commands over serial to Arduino. Provide a calibration routine to calibrate motor PWM→speed.
 - Modular: SerialComm, PIDController, MotionPlanner, VisionDetector, Calibration

Expected Arduino serial protocol:
  - CMD:MOVE,<DIR>,<SPEED>,<DURATION_MS>\n
  - CMD:STOP\n
  - CMD:CAL_TEST,MOTOR_INDEX,PWM_MS,DURATION_MS\n

import time
import math
import threading
from collections import deque
import serial
import serial.tools.list_ports
import numpy as np
import cv2

# ---------- CONFIG ----------
SERIAL_BAUD = 115200
SERIAL_PORT = None   # None -> auto-detect
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
HISTORY_LEN = 5

# Motion thresholds (pixels)
X_TOL = FRAME_WIDTH * 0.05
Y_TOL = FRAME_HEIGHT * 0.05

STOP_AREA_THRESHOLD = 5000.0  # tune

# PID gains (tune)
Kp_x, Ki_x, Kd_x = 0.008, 0.0002, 0.0005
Kp_y, Ki_y, Kd_y = 0.008, 0.0002, 0.0005

MAX_SPEED = 200
CAMERA_INDEX = 0
YOLO_MODEL_PATH = "Training&Testing/best.pt"

# ---------------- Serial ----------------
class SerialComm:
    def __init__(self, port=None, baud=SERIAL_BAUD, timeout=1.0):
        self.port = port or self._auto_detect_port()
        if self.port is None:
            raise RuntimeError("No serial port found.")
        self.ser = serial.Serial(self.port, baud, timeout=timeout)
        self.lock = threading.Lock()
        self._running = True
        self._read_thread = threading.Thread(target=self._reader, daemon=True)
        self._read_thread.start()
        self.incoming_lines = deque(maxlen=200)

    def _auto_detect_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if "USB" in p.description.upper() or "SERIAL" in p.description.upper():
                return p.device
        if ports:
            return ports[0].device
        return None

    def _reader(self):
        while self._running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        self.incoming_lines.append(line)
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.1)

    def send_command(self, cmd: str):
        with self.lock:
            self.ser.write((cmd.strip() + "\n").encode())

    def get_lines(self):
        lines = list(self.incoming_lines)
        self.incoming_lines.clear()
        return lines

    def close(self):
        self._running = False
        time.sleep(0.05)
        try:
            self.ser.close()
        except Exception:
            pass

# ---------------- PID ----------------
class PIDController:
    def __init__(self, kp, ki, kd, out_min=-MAX_SPEED, out_max=MAX_SPEED):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self._int = 0.0
        self._last = None
        self._last_t = None

    def reset(self):
        self._int = 0.0
        self._last = None
        self._last_t = None

    def update(self, error, t=None):
        t = t or time.time()
        dt = 0.0 if self._last_t is None else (t - self._last_t)
        P = self.kp * error
        if dt > 0: self._int += error * dt
        I = self.ki * self._int
        D = 0.0
        if self._last is not None and dt > 0:
            D = self.kd * ((error - self._last) / dt)
        self._last, self._last_t = error, t
        out = max(self.out_min, min(self.out_max, P+I+D))
        return out

# ---------------- Motion Planner ----------------
class MotionPlanner:
    def __init__(self, serial_comm, frame_w, frame_h, x_tol, y_tol, stop_area):
        self.ser = serial_comm
        self.center_x, self.center_y = frame_w//2, frame_h//2
        self.x_history = deque(maxlen=HISTORY_LEN)
        self.y_history = deque(maxlen=HISTORY_LEN)
        self.a_history = deque(maxlen=HISTORY_LEN)
        self.cmd_history = deque(maxlen=HISTORY_LEN)
        self.pid_x = PIDController(Kp_x, Ki_x, Kd_x)
        self.pid_y = PIDController(Kp_y, Ki_y, Kd_y)
        self.x_tol, self.y_tol = x_tol, y_tol
        self.stop_area = stop_area
        self.last_sent_time = 0.0
        self.min_command_interval = 0.05

    def update_from_detection(self, cx, cy, area):
        if cx is None:
            if self.x_history:
                self.x_history.append(self.x_history[-1])
                self.y_history.append(self.y_history[-1])
                self.a_history.append(self.a_history[-1])
            else:
                self.x_history.append(self.center_x)
                self.y_history.append(self.center_y)
                self.a_history.append(0.0)
        else:
            self.x_history.append(int(cx))
            self.y_history.append(int(cy))
            self.a_history.append(float(area))

    def _smoothed(self):
        cx = int(np.mean(self.x_history)) if self.x_history else self.center_x
        cy = int(np.mean(self.y_history)) if self.y_history else self.center_y
        a = float(np.mean(self.a_history)) if self.a_history else 0.0
        return cx, cy, a

    def decide_and_send(self, until_center=False):
        cx, cy, a = self._smoothed()
        dx = cx - self.center_x
        dy = self.center_y - cy
        vx = self.pid_x.update(-dx)
        vy = self.pid_y.update(-dy)
        speed_x, speed_y = abs(int(vx)), abs(int(vy))
        speed_x = min(MAX_SPEED, speed_x)
        speed_y = min(MAX_SPEED, speed_y)

        if a >= self.stop_area and a > 0:
            cmd = ("STOP", 0, 0)
            self._send_cmd_now(cmd)
            return cmd

        if abs(dx) <= self.x_tol and abs(dy) <= self.y_tol:
            cmd = ("STOP", 0, 0)
            self._send_cmd_now(cmd)
            return cmd

        norm_x = abs(dx)/(FRAME_WIDTH/2)
        norm_y = abs(dy)/(FRAME_HEIGHT/2)
        if norm_x >= norm_y:
            direction = "RIGHT" if dx > 0 else "LEFT"
            speed = speed_x or 40
            cmd = (direction, speed, 0 if until_center else 300)
            self._send_cmd_now(cmd)
            return cmd
        else:
            direction = "FORWARD" if dy > 0 else "BACKWARD"
            speed = speed_y or 40
            cmd = (direction, speed, 0 if until_center else 300)
            self._send_cmd_now(cmd)
            return cmd

    def _send_cmd_now(self, cmd_tuple):
        dir_name, speed, duration_ms = cmd_tuple
        now = time.time()
        if now - self.last_sent_time < self.min_command_interval:
            return
        if dir_name == "STOP":
            line = "CMD:STOP"
        else:
            line = f"CMD:MOVE,{dir_name},{int(speed)},{int(duration_ms)}"
        self.ser.send_command(line)
        self.cmd_history.append(line)
        self.last_sent_time = now

# ---------------- Vision Detector ----------------
class VisionDetector:
    def __init__(self, model_path, camera_idx, frame_w, frame_h):
        self.model = None
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
        except Exception as e:
            print("Warning: YOLO failed to load.", e)
            self.model = None
        self.cap = cv2.VideoCapture(camera_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)

    def read_frame(self):
        ret, frame = self.cap.read()
        return (frame, True) if ret else (None, None)

    def detect_best_circle(self, frame, yolo_box=None):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower1, upper1 = np.array([0,100,70]), np.array([10,255,255])
        lower2, upper2 = np.array([170,100,70]), np.array([180,255,255])
        mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1),
                              cv2.inRange(hsv, lower2, upper2))
        if yolo_box:
            x1,y1,x2,y2 = yolo_box
            search_img = mask[y1:y2, x1:x2]
            base_x, base_y = x1,y1
        else:
            search_img, base_x, base_y = mask,0,0
        blur = cv2.GaussianBlur(search_img,(7,7),0)
        contours,_=cv2.findContours(blur,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        best=(None,None,0.0)
        for c in contours:
            area=cv2.contourArea(c)
            if area<50: continue
            (x,y),r=cv2.minEnclosingCircle(c)
            if r<3: continue
            cx,cy=int(x+base_x),int(y+base_y)
            perimeter=cv2.arcLength(c,True)
            if perimeter==0: continue
            circularity=4*math.pi*(area/(perimeter*perimeter+1e-8))
            score=area*circularity
            if score>best[2]: best=(cx,cy,area)
        if best[0] is None: return None,None,0.0
        return best

    def detect_yolo_then_circle(self, frame):
        if self.model is None:
            return self.detect_best_circle(frame,None)
        results=self.model(frame,verbose=False)
        best=(None,None,0.0)
        for r in results:
            for box in r.boxes:
                x1,y1,x2,y2=map(int,box.xyxy[0].tolist())
                cx,cy,area=self.detect_best_circle(frame,(x1,y1,x2,y2))
                if area>best[2]: best=(cx,cy,area)
        if best[0] is None:
            return self.detect_best_circle(frame,None)
        return best

    def release(self):
        self.cap.release()

# ---------------- Calibration ----------------
def calibration_routine(serial_comm, motors=3):
    print("Starting calibration routine.")
    pwm_test=150; dur_ms=1000; speed_factors=[]
    for m in range(motors):
        cmd=f"CMD:CAL_TEST,{m},{pwm_test},{dur_ms}"
        print("Sending:",cmd)
        serial_comm.send_command(cmd)
        end_t=time.time()+2.0; counts=None
        while time.time()<end_t:
            for L in serial_comm.get_lines():
                if L.startswith("OK:ENC"):
                    parts=L.split(",")
                    try:
                        idx=int(parts[1]); cnt=int(parts[2])
                        if idx==m: counts=cnt; break
                    except: pass
            if counts is not None: break
            time.sleep(0.05)
        if counts is None:
            print(f"Motor {m}: no reply.")
            speed_factors.append(None)
        else:
            print(f"Motor {m}: encoder counts = {counts}")
            speed_factors.append(counts)
    valid=[s for s in speed_factors if s]
    if valid:
        maxc=max(valid)
        factors=[(s/maxc if s else None) for s in speed_factors]
    else:
        factors=[None]*motors
    print("Calibration factors:",factors)
    return factors

# ---------------- Main ----------------
def main(app_mode="run"):
    try:
        ser=SerialComm(port=SERIAL_PORT,baud=SERIAL_BAUD)
        print("Serial port:",ser.port)
    except Exception as e:
        print("Serial init failed:",e); return
    planner=MotionPlanner(ser,FRAME_WIDTH,FRAME_HEIGHT,X_TOL,Y_TOL,STOP_AREA_THRESHOLD)
    detector=VisionDetector(YOLO_MODEL_PATH,CAMERA_INDEX,FRAME_WIDTH,FRAME_HEIGHT)
    try:
        if app_mode=="calibrate":
            factors=calibration_routine(ser,motors=3)
            print("Calibration complete:",factors); return
        print("Main loop. Press q to quit.")
        while True:
            frame,ok=detector.read_frame()
            if frame is None: print("Camera read failed."); break
            cx,cy,area=detector.detect_yolo_then_circle(frame)
            planner.update_from_detection(cx,cy,area)
            cmd=planner.decide_and_send(until_center=False)
            sm_cx,sm_cy,sm_area=planner._smoothed()
            txt=f"sm_cx={sm_cx} sm_cy={sm_cy} area={int(sm_area)} cmd={cmd}"
            cv2.putText(frame,txt,(10,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,0),1)
            if cx is not None: cv2.circle(frame,(int(cx),int(cy)),6,(0,255,0),2)
            cv2.circle(frame,(planner.center_x,planner.center_y),4,(255,0,0),-1)
            cv2.imshow("Tracker",frame)
            key=cv2.waitKey(1)&0xFF
            if key==ord('q'): break
            elif key==ord('c'): calibration_routine(ser,motors=3)
            elif key==ord('s'):
                for L in ser.get_lines(): print("SERIAL:",L)
    finally:
        detector.release(); ser.close(); cv2.destroyAllWindows()

if __name__=="__main__":
    main(app_mode="run")
