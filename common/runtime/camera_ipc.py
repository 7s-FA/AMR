"""Latest BGR8 camera frame and profile control without ROS or DDS."""
import mmap,os,struct,time,socket,json,fcntl
from pathlib import Path
import numpy as np
HEADER=struct.Struct('<QQQIIIIII');OFFSET=128;MAX_BYTES=640*480*3

def paths(robot):
 if robot not in ('burger1','burger2'):raise ValueError('Unknown robot')
 runtime=Path(os.environ.get('XDG_RUNTIME_DIR','/run/user/'+str(os.getuid())))
 return runtime/(robot+'-camera.frames'),runtime/(robot+'-camera-control.sock')

def profile(robot,value='status',timeout=2.):
 if value not in ('active','idle','status'):raise ValueError('Unknown camera profile')
 _,path=paths(robot)
 with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as conn:
  conn.settimeout(timeout);conn.connect(str(path));conn.sendall((value+'\n').encode());raw=b''
  while b'\n' not in raw and len(raw)<4096:
   data=conn.recv(4096)
   if not data:raise RuntimeError('Camera control disconnected')
   raw+=data
  result=json.loads(raw)
 if result.get('ros_video') is not False:raise RuntimeError('Native camera transport not confirmed')
 return result

class SharedFrameSource:
 def __init__(self,robot,frame_id=None,max_age_s=.5,path=None):
  self.path=Path(path) if path else paths(robot)[0];self.frame_id=frame_id or robot+'_camera_optical_frame';self.max_age_s=max_age_s
  self.file=self.map=None;self.last_stamp=0;self.accepted_frames=self.stale_frames=0
 def _open(self):
  if self.map is not None:return
  self.file=self.path.open('rb');size=os.fstat(self.file.fileno()).st_size
  if size!=OFFSET+MAX_BYTES:self.file.close();self.file=None;raise RuntimeError('Invalid native frame size')
  self.map=mmap.mmap(self.file.fileno(),size,access=mmap.ACCESS_READ)
 def read(self,timeout):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   try:self._open()
   except (OSError,RuntimeError):time.sleep(.01);continue
   try:fcntl.flock(self.file.fileno(),fcntl.LOCK_SH|fcntl.LOCK_NB)
   except BlockingIOError:time.sleep(.005);continue
   try:
    seq,stamp,boot,width,height,channels,stride,nbytes,version=HEADER.unpack_from(self.map)
    if seq and not seq%2 and stamp>self.last_stamp:
     if (width,height,channels,stride,nbytes,version)!=(640,480,3,1920,MAX_BYTES,1):raise RuntimeError('Camera geometry/version changed')
     age=(time.time_ns()-stamp)/1e9
     if not -.1<=age<=self.max_age_s:
      self.stale_frames+=1;self.last_stamp=stamp
     else:
      image=np.ndarray((480,640,3),dtype=np.uint8,buffer=self.map,offset=OFFSET).copy()
      if struct.unpack_from('<Q',self.map)[0]==seq:
       self.last_stamp=stamp;self.accepted_frames+=1
       return image,{'source_stamp':{'sec':stamp//10**9,'nanosec':stamp%10**9},'received_monotonic':time.monotonic(),'frame_id':self.frame_id,'frame_sequence':seq//2,'transport':'native_shared_memory'}
   finally:fcntl.flock(self.file.fileno(),fcntl.LOCK_UN)
   time.sleep(.005)
  raise RuntimeError('No fresh native camera frame')
 def close(self):
  if self.map is not None:self.map.close();self.map=None
  if self.file is not None:self.file.close();self.file=None
