"""Read-only bounded JPEG preview; networking never runs in the control process."""
import json,time,threading,socket
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

class LatestVideo:
 def __init__(self,robot):
  self.robot=robot;self.condition=threading.Condition();self.jpeg=None;self.metadata={};self.version=0;self.clients=0;self.snapshot_until=0.
 def requested(self):
  with self.condition:return self.clients>0 or time.monotonic()<self.snapshot_until
 def update(self,jpeg,metadata):
  with self.condition:
   self.jpeg=jpeg;self.metadata=metadata;self.version+=1;self.condition.notify_all()
 def wait(self,previous,timeout=1.):
  with self.condition:
   self.condition.wait_for(lambda:self.version>previous,timeout)
   return self.version,self.jpeg,self.metadata.copy()

def serve(video,host='0.0.0.0',port=8085):
 class Handler(BaseHTTPRequestHandler):
  protocol_version='HTTP/1.1'
  def log_message(self,*_):pass
  def do_GET(self):
   if self.path=='/health':
    stamp=video.metadata.get('source_stamp',{});age=time.time()-stamp.get('sec',0)-stamp.get('nanosec',0)/1e9 if stamp else None
    body=json.dumps(dict(robot=video.robot,transport='HTTP_JPEG',ros_video=False,clients=video.clients,frame_age_s=age)).encode();self.send_response(200);self.send_header('Access-Control-Allow-Origin','*');self.send_header('Cache-Control','no-store');self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body);return
   if self.path not in ('/stream.mjpg','/snapshot.jpg'):self.send_error(404);return
   self.connection.settimeout(2.)
   if self.path=='/snapshot.jpg':
    with video.condition:video.snapshot_until=time.monotonic()+2.;before=video.version
    version,jpeg,metadata=video.wait(before,1.5)
    if jpeg is None or version<=before:self.send_error(503);return
    self.send_response(200);self.send_header('Content-Type','image/jpeg');self.send_header('Content-Length',str(len(jpeg)));self.end_headers();self.wfile.write(jpeg);return
   with video.condition:
    if video.clients>=2:self.send_error(503,'Maximum two display clients');return
    video.clients+=1
   try:
    self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.send_header('Cache-Control','no-store');self.send_header('Connection','close');self.end_headers()
    last=-1;last_fresh=time.monotonic()
    while True:
     version,jpeg,metadata=video.wait(last,1.)
     if time.monotonic()-last_fresh>5.:break
     if jpeg is None or version==last:continue
     last=version;stamp=metadata.get('source_stamp',{});ns=stamp.get('sec',0)*10**9+stamp.get('nanosec',0)
     if not -.1<=(time.time_ns()-ns)/1e9<=2.:continue
     last_fresh=time.monotonic()
     header=f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\nX-Capture-Time-Ns: {ns}\r\nX-Robot: {video.robot}\r\n\r\n'.encode()
     self.wfile.write(header);self.wfile.write(jpeg);self.wfile.write(b'\r\n');self.wfile.flush()
   except (OSError,socket.timeout):pass
   finally:
    with video.condition:video.clients-=1
    self.close_connection=True
 class Server(ThreadingHTTPServer):daemon_threads=True;request_queue_size=4
 server=Server((host,port),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();return server
