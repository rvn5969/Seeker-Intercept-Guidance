#include "intercept/config.hpp"
#include "intercept/guidance_stack.hpp"
#include "intercept/packets.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

using namespace intercept;

int main(int argc,char** argv){
  try{
    const auto cfg=loadConfig(argc>1?argv[1]:"config/experiment.json");
    const int fd=::socket(AF_INET,SOCK_DGRAM,0); if(fd<0){ perror("socket"); return 1; }
    sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_addr.s_addr=INADDR_ANY; addr.sin_port=htons(cfg.hil.udp_port);
    if(::bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0){ perror("bind"); close(fd); return 1; }
    GuidanceStack stack(cfg); std::cout<<"HIL controller listening on UDP "<<cfg.hil.udp_port<<"\n";
    for(;;){
      SensorPacket in{}; sockaddr_in peer{}; socklen_t n=sizeof(peer);
      const auto bytes=recvfrom(fd,&in,sizeof(in),0,reinterpret_cast<sockaddr*>(&peer),&n);
      if(bytes!=static_cast<ssize_t>(sizeof(in))||in.magic!=0x5345454B) continue;
      SeekerMeasurement z{in.measurement_timestamp,in.range,in.bearing,in.own_speed,in.own_heading,in.own_lateral_accel,in.valid!=0};
      const auto c=stack.update(z,in.current_timestamp); CommandPacket out; out.timestamp=c.timestamp; out.lateral_accel=c.lateral_accel; out.valid=c.valid?1:0;
      sendto(fd,&out,sizeof(out),0,reinterpret_cast<sockaddr*>(&peer),n);
    }
  } catch(const std::exception& e){ std::cerr<<"error: "<<e.what()<<'\n'; return 1; }
}
