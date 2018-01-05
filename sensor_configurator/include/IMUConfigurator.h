#include "SensorConfigurator.h"
#include <vector>

class IMUConfigurator : public SensorConfigurator{
public:
  IMUConfigurator();
  virtual void init();
  virtual void setParameterData(ros::NodeHandle& nh);
  virtual bool ok();
private:
  virtual bool parse();

  //std::map으로 대체 가능
  std::vector<std::string> paramName;
  std::vector<double> imuData;
  //remember that SensorConfigurator has a communicator member : _ptr
};
