#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <vector>

template <int N>
class VectorConcate : public RTT::TaskContext {
public:
  VectorConcate(const std::string & name) : TaskContext(name, PreOperational) {
    for(size_t i = 0; i < N; i++) {
      char port_name[10];
      sprintf(port_name, "In%zu", i);
      this->ports()->addPort(port_name, port_inputs_[i]);
    }
    
    this->ports()->addPort("Out", port_output_);
  }

  ~VectorConcate(){
  }

  bool configureHook() {
    size_t size = 0;
    for(size_t i = 0; i < N; i++) {
      port_inputs_[i].getDataSample(inputs_[i]);
      size += inputs_[i].size();
    }
    
    output_.resize(size);
    port_output_.setDataSample(output_);
    
    return true;
  }

  void updateHook() {
    size_t k = 0;
    for(size_t i = 0; i < N; i++) {
      port_inputs_[i].read(inputs_[i]);
      for(size_t j = 0; j < inputs_[i].size(); j++) {
        output_[k++] = inputs_[i][j];
      }
    }
    
    port_output_.write(output_);
  }

private:

  RTT::InputPort<std::vector<double> > port_inputs_[N];
  
  RTT::OutputPort<std::vector<double> > port_output_;

  std::vector<double> inputs_[N];
  std::vector<double > output_;
};
