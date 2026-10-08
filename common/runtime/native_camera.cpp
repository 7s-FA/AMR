// libcamera capture without ROS, DDS, ImageTransport or network image publishers.
#include <libcamera/libcamera.h>
#include <libcamera/formats.h>
#include <libcamera/control_ids.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <ctime>
#include <atomic>
#include <array>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>
#include <string>
#include <algorithm>
static std::atomic<bool> quit{false};
static uint64_t clock_ns(clockid_t c){timespec t{};clock_gettime(c,&t);return uint64_t(t.tv_sec)*1000000000+t.tv_nsec;}
struct alignas(64) FrameHeader {uint64_t sequence=0,epoch_ns=0,boot_ns=0;uint32_t width=640,height=480,channels=3,stride=1920,bytes=921600,version=1;char padding[80]{};};
static_assert(sizeof(FrameHeader)==128);
struct Mapping {void* address=MAP_FAILED;size_t length=0,offset=0;};
class Capture {
 std::shared_ptr<libcamera::Camera> camera;std::unique_ptr<libcamera::CameraConfiguration> config;
 std::unique_ptr<libcamera::FrameBufferAllocator> allocator;libcamera::Stream* stream=nullptr;
 std::vector<std::unique_ptr<libcamera::Request>> requests;std::map<libcamera::FrameBuffer*,Mapping> maps;
 int frame_fd=-1,lock_fd=-1,server=-1;void* shared=MAP_FAILED;size_t shared_size=128+640*480*3;
 std::atomic<int64_t> duration{500000};std::atomic<bool> running{false};std::atomic<uint64_t> frames{0},copy_cpu_ns{0};
 std::string output,socket_path;uint64_t sequence=0;
 void controls(libcamera::ControlList& c){std::array<int64_t,2> limits{duration.load(),duration.load()};c.set(libcamera::controls::FrameDurationLimits,libcamera::Span<const int64_t,2>(limits));}
 public:
 Capture(libcamera::CameraManager& manager,std::string out,std::string sock):output(out),socket_path(sock){
  lock_fd=open((output+".lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0600);if(lock_fd<0||flock(lock_fd,LOCK_EX|LOCK_NB))throw std::runtime_error("Camera capture already owned");
  if(manager.cameras().empty())throw std::runtime_error("No libcamera camera");camera=manager.cameras().at(0);
  if(camera->acquire())throw std::runtime_error("Camera acquire failed");
  config=camera->generateConfiguration({libcamera::StreamRole::Viewfinder,libcamera::StreamRole::Raw});
  if(!config||config->size()!=2)throw std::runtime_error("Camera streams unavailable");
  config->orientation=libcamera::Orientation::Rotate0;
  config->at(0).pixelFormat=libcamera::formats::RGB888; // DRM little-endian RGB888 bytes are BGR8.
  config->at(0).size={640,480};config->at(1).size={1640,1232};
  if(config->validate()==libcamera::CameraConfiguration::Invalid||config->at(0).size!=libcamera::Size{640,480}||config->at(0).pixelFormat!=libcamera::formats::RGB888||config->at(1).size!=libcamera::Size{1640,1232})throw std::runtime_error("Requested calibrated geometry was adjusted; refusing capture");
  if(camera->configure(config.get()))throw std::runtime_error("Camera configure failed");
  stream=config->at(0).stream();allocator=std::make_unique<libcamera::FrameBufferAllocator>(camera);
  size_t count=100;
  for(auto& sc:*config){if(allocator->allocate(sc.stream())<0)throw std::runtime_error("Camera buffer allocation failed");count=std::min(count,allocator->buffers(sc.stream()).size());}
  for(const auto& buffer:allocator->buffers(stream)){
   if(buffer->planes().size()!=1)throw std::runtime_error("Packed BGR8 plane required");const auto& plane=buffer->planes()[0];
   if(plane.length < size_t(config->at(0).stride)*480)throw std::runtime_error("Camera plane shorter than calibrated image");
   size_t page=sysconf(_SC_PAGE_SIZE),base=plane.offset/page*page,off=plane.offset-base;
   void* memory=mmap(nullptr,plane.length+off,PROT_READ,MAP_SHARED,plane.fd.get(),base);
   if(memory==MAP_FAILED)throw std::runtime_error("Camera mmap failed");maps[buffer.get()]={memory,plane.length+off,off};
  }
  for(size_t i=0;i<count;++i){auto r=camera->createRequest(i);if(!r)throw std::runtime_error("Create request failed");for(auto& sc:*config)if(r->addBuffer(sc.stream(),allocator->buffers(sc.stream())[i].get()))throw std::runtime_error("Add buffer failed");requests.push_back(std::move(r));}
  frame_fd=open(output.c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0600);if(frame_fd<0||ftruncate(frame_fd,shared_size))throw std::runtime_error("Shared frame file unavailable");
  shared=mmap(nullptr,shared_size,PROT_READ|PROT_WRITE,MAP_SHARED,frame_fd,0);if(shared==MAP_FAILED)throw std::runtime_error("Shared frame mmap failed");
  std::memset(shared,0,128);new(shared)FrameHeader{};
  server=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);sockaddr_un address{};address.sun_family=AF_UNIX;if(socket_path.size()>=sizeof(address.sun_path))throw std::runtime_error("Control socket path too long");std::strcpy(address.sun_path,socket_path.c_str());unlink(socket_path.c_str());
  if(bind(server,reinterpret_cast<sockaddr*>(&address),sizeof(address))||listen(server,4))throw std::runtime_error("Camera control socket failed");chmod(socket_path.c_str(),0600);
  camera->requestCompleted.connect(this,&Capture::complete);
 }
 void start(){libcamera::ControlList c(camera->controls());controls(c);if(camera->start(&c))throw std::runtime_error("Camera start failed");running=true;for(auto& r:requests){controls(r->controls());if(camera->queueRequest(r.get()))throw std::runtime_error("Camera queue failed");}std::cout<<"NATIVE_CAMERA_READY 640x480 BGR8 sensor=1640x1232 no_ROS_video"<<std::endl;}
 void complete(libcamera::Request* request){
  if(request->status()==libcamera::Request::RequestCancelled||!running)return;
  auto* buffer=request->buffers().at(stream);auto& map=maps.at(buffer);auto* h=static_cast<FrameHeader*>(shared);
  auto sensor=request->metadata().get(libcamera::controls::SensorTimestamp);uint64_t capture_ns=sensor?uint64_t(*sensor):buffer->metadata().timestamp;
  uint64_t boot=clock_ns(CLOCK_BOOTTIME),wall=clock_ns(CLOCK_REALTIME);uint64_t epoch=wall-(boot>capture_ns?boot-capture_ns:0);
  uint64_t cpu=clock_ns(CLOCK_THREAD_CPUTIME_ID);
  if(flock(frame_fd,LOCK_EX|LOCK_NB)==0){ // Nonblocking process-shared lock; drop a busy frame.
  __atomic_store_n(&h->sequence,++sequence,__ATOMIC_SEQ_CST); // odd: writing; one latest frame, no FIFO.
  h->epoch_ns=epoch;h->boot_ns=capture_ns;
  const auto* src=static_cast<uint8_t*>(map.address)+map.offset;auto* dst=static_cast<uint8_t*>(shared)+128;
  for(unsigned y=0;y<480;++y)std::memcpy(dst+y*1920,src+y*config->at(0).stride,1920);
  __atomic_store_n(&h->sequence,++sequence,__ATOMIC_RELEASE);++frames;copy_cpu_ns+=clock_ns(CLOCK_THREAD_CPUTIME_ID)-cpu;
  flock(frame_fd,LOCK_UN);
  }
  request->reuse(libcamera::Request::ReuseBuffers);controls(request->controls());if(running&&camera->queueRequest(request))quit=true;
 }
 void loop(){uint64_t last=clock_ns(CLOCK_MONOTONIC),old_frames=0,old_cpu=0;
  while(!quit){pollfd p{server,POLLIN,0};if(poll(&p,1,100)>0){int fd=accept4(server,nullptr,nullptr,SOCK_CLOEXEC);if(fd>=0){timeval timeout{1,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));char raw[64]{};ssize_t got=recv(fd,raw,sizeof(raw)-1,0);std::string command=got>0?std::string(raw,size_t(got)):"";
    if(command=="active\n")duration=66667;else if(command=="idle\n")duration=500000;
    std::string reply="{\"camera_profile\":\""+std::string(duration==66667?"active":"idle")+"\",\"frame_duration_us\":"+std::to_string(duration.load())+",\"frames\":"+std::to_string(frames.load())+",\"video_transport\":\"local_shared_memory\",\"ros_video\":false}\n";
    send(fd,reply.data(),reply.size(),MSG_NOSIGNAL);close(fd);
   }}
   uint64_t now=clock_ns(CLOCK_MONOTONIC);if(now-last>=5000000000ULL){auto n=frames.load(),c=copy_cpu_ns.load();std::cout<<"CAMERA_DATA_PROFILE {\"window_s\":"<<(now-last)/1e9<<",\"frames\":"<<n-old_frames<<",\"copy_cpu_ms\":"<<(c-old_cpu)/1e6<<"}"<<std::endl;last=now;old_frames=n;old_cpu=c;}
  }
 }
 ~Capture(){running=false;if(camera){camera->stop();camera->requestCompleted.disconnect(this);}for(auto& [_,m]:maps)if(m.address!=MAP_FAILED)munmap(m.address,m.length);requests.clear();allocator.reset();if(camera)camera->release();if(shared!=MAP_FAILED)munmap(shared,shared_size);if(frame_fd>=0)close(frame_fd);if(server>=0)close(server);if(lock_fd>=0)close(lock_fd);unlink(socket_path.c_str());}
};
int main(int argc,char**argv){if(argc!=3)return 2;signal(SIGINT,[](int){quit=true;});signal(SIGTERM,[](int){quit=true;});try{libcamera::CameraManager manager;if(manager.start())throw std::runtime_error("Camera manager start failed");{Capture capture(manager,argv[1],argv[2]);capture.start();capture.loop();}manager.stop();}catch(const std::exception& e){std::cerr<<"Native camera failed: "<<e.what()<<std::endl;return 1;}return 0;}
