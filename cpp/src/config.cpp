#include "intercept/config.hpp"
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>
#include <variant>
#include <vector>

namespace intercept {
namespace {

struct Json {
  using Object=std::map<std::string,Json>;
  using Array=std::vector<Json>;
  std::variant<std::nullptr_t,bool,double,std::string,Array,Object> value;

  const Object& object() const { return std::get<Object>(value); }
  const Array& array() const { return std::get<Array>(value); }
  const Json& at(const std::string& key) const {
    const auto& o=object(); auto it=o.find(key);
    if(it==o.end()) throw std::runtime_error("Missing JSON key: "+key);
    return it->second;
  }
  double number() const { return std::get<double>(value); }
  int integer() const { return static_cast<int>(std::llround(number())); }
  bool boolean() const { return std::get<bool>(value); }
  const std::string& string() const { return std::get<std::string>(value); }
};

class Parser {
public:
  explicit Parser(std::string text):text_(std::move(text)){}
  Json parse(){ auto j=value(); skip(); if(pos_!=text_.size()) fail("Trailing data"); return j; }
private:
  Json value(){
    skip(); if(pos_>=text_.size()) fail("Unexpected end of file");
    const char c=text_[pos_];
    if(c=='{') return object();
    if(c=='[') return array();
    if(c=='\"') return Json{string()};
    if(c=='t'){ literal("true"); return Json{true}; }
    if(c=='f'){ literal("false"); return Json{false}; }
    if(c=='n'){ literal("null"); return Json{nullptr}; }
    if(c=='-' || std::isdigit(static_cast<unsigned char>(c))) return Json{number()};
    fail("Unexpected token"); return Json{nullptr};
  }
  Json object(){
    Json::Object o; expect('{'); skip(); if(consume('}')) return Json{o};
    for(;;){
      skip(); if(pos_>=text_.size() || text_[pos_]!='\"') fail("Expected object key");
      auto key=string(); expect(':'); o.emplace(std::move(key),value()); skip();
      if(consume('}')) break;
      expect(',');
    }
    return Json{o};
  }
  Json array(){
    Json::Array a; expect('['); skip(); if(consume(']')) return Json{a};
    for(;;){ a.push_back(value()); skip(); if(consume(']')) break; expect(','); }
    return Json{a};
  }
  std::string string(){
    expect('\"'); std::string out;
    while(pos_<text_.size()){
      char c=text_[pos_++]; if(c=='\"') return out;
      if(c=='\\'){
        if(pos_>=text_.size()) fail("Bad string escape");
        char e=text_[pos_++];
        switch(e){
          case '\"': out+='\"'; break; case '\\': out+='\\'; break; case '/': out+='/'; break;
          case 'b': out+='\b'; break; case 'f': out+='\f'; break; case 'n': out+='\n'; break;
          case 'r': out+='\r'; break; case 't': out+='\t'; break;
          default: fail("Unsupported JSON escape");
        }
      } else out+=c;
    }
    fail("Unterminated string"); return {};
  }
  double number(){
    const auto start=pos_;
    if(text_[pos_]=='-') ++pos_;
    while(pos_<text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_;
    if(pos_<text_.size() && text_[pos_]=='.'){ ++pos_; while(pos_<text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
    if(pos_<text_.size() && (text_[pos_]=='e'||text_[pos_]=='E')){
      ++pos_; if(pos_<text_.size() && (text_[pos_]=='+'||text_[pos_]=='-')) ++pos_;
      while(pos_<text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_;
    }
    return std::stod(text_.substr(start,pos_-start));
  }
  void literal(const char* s){ while(*s){ if(pos_>=text_.size()||text_[pos_++]!=*s++) fail("Bad literal"); } }
  void expect(char c){ skip(); if(pos_>=text_.size()||text_[pos_++]!=c) fail(std::string("Expected '")+c+"'"); }
  bool consume(char c){ skip(); if(pos_<text_.size()&&text_[pos_]==c){++pos_;return true;} return false; }
  void skip(){ while(pos_<text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
  [[noreturn]] void fail(const std::string& msg) const { throw std::runtime_error(msg+" near byte "+std::to_string(pos_)); }
  std::string text_; std::size_t pos_{};
};

Range range(const Json& j){ const auto& a=j.array(); if(a.size()!=2) throw std::runtime_error("Range must have two values"); return {a[0].number(),a[1].number()}; }
Vec2 vec2(const Json& j){ const auto& a=j.array(); if(a.size()!=2) throw std::runtime_error("Vector must have two values"); return {a[0].number(),a[1].number()}; }
std::optional<std::uint64_t> seed(const Json& j){
  if(std::holds_alternative<std::string>(j.value)){
    if(j.string()!="random") throw std::runtime_error("Seed string must be 'random'");
    return std::nullopt;
  }
  const double x=j.number(); if(x<0) throw std::runtime_error("Seed must be nonnegative");
  return static_cast<std::uint64_t>(x);
}
void positive(double x,const char* name){ if(!(x>0)) throw std::runtime_error(std::string(name)+" must be > 0"); }
void validRange(Range r,const char* name){ if(r.lo>r.hi) throw std::runtime_error(std::string(name)+" lower bound exceeds upper bound"); }

} // namespace

ExperimentConfig loadConfig(const std::string& path){
  std::ifstream f(path); if(!f) throw std::runtime_error("Cannot open config: "+path);
  std::string text((std::istreambuf_iterator<char>(f)),{}); const auto root=Parser(std::move(text)).parse();
  ExperimentConfig c;

  const auto& s=root.at("simulation");
  c.simulation={s.at("truth_dt_s").number(),s.at("guidance_dt_s").number(),s.at("max_time_s").number(),s.at("intercept_radius_m").number()};

  const auto& i=root.at("interceptor");
  c.interceptor.speed=i.at("speed_mps").number(); c.interceptor.initial_position=vec2(i.at("initial_position_m"));
  c.interceptor.initial_heading=i.at("initial_heading_rad").number(); c.interceptor.initial_accel=i.at("initial_lateral_accel_mps2").number();
  c.interceptor.max_accel=i.at("max_lateral_accel_mps2").number(); c.interceptor.max_slew=i.at("max_slew_mps3").number();
  c.interceptor.actuator_tau=i.at("actuator_tau_s").number();

  const auto& t=root.at("target");
  c.target.speed=t.at("speed_mps").number(); c.target.downrange=range(t.at("initial_downrange_m"));
  c.target.lateral=range(t.at("initial_lateral_offset_m")); c.target.heading_center=t.at("initial_heading_center_rad").number();
  c.target.heading_offset=range(t.at("initial_heading_offset_rad")); c.target.accel=range(t.at("lateral_accel_mps2"));

  const auto& q=root.at("seeker");
  c.seeker={q.at("update_rate_hz").number(),q.at("range_sigma_m").number(),q.at("bearing_sigma_rad").number(),
            q.at("dropout_probability").number(),range(q.at("latency_s"))};

  const auto& e=root.at("estimator");
  c.estimator={e.at("initial_position_variance_m2").number(),e.at("initial_velocity_variance_m2ps2").number(),
               e.at("process_accel_sigma_mps2").number(),e.at("target_accel_smoothing_alpha").number(),
               e.at("target_accel_max_mps2").number()};

  const auto& g=root.at("guidance"); c.guidance.mode=g.at("mode").string();
  c.guidance.pn.navigation_constant=g.at("pn").at("navigation_constant").number();
  const auto& p=g.at("predictive");
  c.guidance.predictive={p.at("prediction_dt_s").number(),p.at("weight_zem").number(),p.at("weight_uncertainty").number(),p.at("weight_accel").number(),
    p.at("weight_slew").number(),p.at("optimizer_step_fraction").number(),p.at("optimizer_min_step_mps2").number(),
    p.at("step_decay").number(),p.at("pn_handoff_tgo_multiplier").number(),p.at("max_time_to_go_s").number(),
    p.at("horizon_steps").integer(),p.at("optimizer_iterations").integer(),p.at("control_blocks").integer(),
    p.at("fallback_to_pn_if_better").boolean()};

  const auto& m=root.at("monte_carlo"); c.monte_carlo={m.at("runs").integer(),m.at("max_workers").integer(),seed(m.at("seed"))};
  const auto& b=root.at("timing_benchmark");
  c.timing.enabled=b.at("enabled").boolean(); c.timing.samples=b.at("samples").integer(); c.timing.seed=seed(b.at("seed"));
  c.timing.relative_x=range(b.at("relative_x_m")); c.timing.relative_y=range(b.at("relative_y_m"));
  c.timing.relative_vx=range(b.at("relative_vx_mps")); c.timing.relative_vy=range(b.at("relative_vy_mps"));

  c.hil.udp_port=root.at("hil").at("udp_port").integer();
  const auto& o=root.at("output"); c.output={o.at("benchmark_csv").string(),o.at("summary_csv").string(),o.at("figure_dir").string()};
  validateConfig(c); return c;
}

void validateConfig(const ExperimentConfig& c){
  positive(c.simulation.truth_dt,"truth_dt_s"); positive(c.simulation.guidance_dt,"guidance_dt_s");
  positive(c.simulation.max_time,"max_time_s"); positive(c.simulation.intercept_radius,"intercept_radius_m");
  positive(c.interceptor.speed,"interceptor speed"); positive(c.interceptor.max_accel,"max acceleration");
  positive(c.interceptor.max_slew,"max slew"); positive(c.interceptor.actuator_tau,"actuator tau");
  positive(c.target.speed,"target speed"); validRange(c.target.downrange,"target downrange"); validRange(c.target.lateral,"target lateral offset");
  validRange(c.target.heading_offset,"target heading offset"); validRange(c.target.accel,"target acceleration");
  positive(c.seeker.update_rate,"seeker update rate"); if(c.seeker.range_sigma<0||c.seeker.bearing_sigma<0) throw std::runtime_error("Sensor sigmas must be nonnegative");
  if(c.seeker.dropout<0||c.seeker.dropout>1) throw std::runtime_error("Dropout probability must lie in [0,1]");
  validRange(c.seeker.latency,"seeker latency");
  if(c.seeker.latency.lo<0) throw std::runtime_error("Latency cannot be negative");
  positive(c.estimator.initial_position_variance,"initial position variance"); positive(c.estimator.initial_velocity_variance,"initial velocity variance");
  if(c.estimator.process_accel_sigma<0) throw std::runtime_error("Process sigma cannot be negative");
  if(c.estimator.accel_smoothing_alpha<0||c.estimator.accel_smoothing_alpha>1) throw std::runtime_error("Smoothing alpha must lie in [0,1]");
  positive(c.estimator.accel_max,"target accel estimate limit"); positive(c.guidance.pn.navigation_constant,"PN navigation constant");
  if(c.guidance.mode!="pn"&&c.guidance.mode!="predictive") throw std::runtime_error("guidance.mode must be 'pn' or 'predictive'");
  const auto& p=c.guidance.predictive; positive(p.prediction_dt,"prediction_dt_s"); positive(p.max_time_to_go,"max_time_to_go_s");
  if(p.horizon_steps<1||p.optimizer_iterations<1||p.control_blocks<1) throw std::runtime_error("Predictive integer parameters must be >= 1");
  if(p.weight_zem<0||p.weight_uncertainty<0||p.weight_accel<0||p.weight_slew<0) throw std::runtime_error("Predictive weights cannot be negative");
  if(p.optimizer_step_fraction<=0||p.optimizer_min_step<=0||p.step_decay<=0||p.step_decay>1) throw std::runtime_error("Invalid optimizer step parameters");
  positive(p.pn_handoff_tgo_multiplier,"PN handoff multiplier"); if(c.monte_carlo.runs<1||c.monte_carlo.max_workers<1) throw std::runtime_error("Monte Carlo settings must be >= 1");
  if(c.timing.samples<1) throw std::runtime_error("Timing samples must be >= 1");
  validRange(c.timing.relative_x,"timing relative x");
  validRange(c.timing.relative_y,"timing relative y"); validRange(c.timing.relative_vx,"timing relative vx"); validRange(c.timing.relative_vy,"timing relative vy");
  if(c.hil.udp_port<1||c.hil.udp_port>65535) throw std::runtime_error("UDP port must lie in [1,65535]");
}

} // namespace intercept
