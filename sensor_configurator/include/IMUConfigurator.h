#include "SensorConfigurator.h"
#include <vector>

class IMUConfigurator : public SensorConfigurator{
public:
  //생성자는 init를 호출하도록 합니다.
  IMUConfigurator();

  //초기화 작업을 수행하는 함수입니다.
  virtual void init();

  //파라미터를 등록하는 함수입니다.
  virtual void setParameterData(ros::NodeHandle& nh);

  //sensor의 데이터가 유효성을 return합니다.
  virtual bool ok();

private:
  //센서별 파싱작업을 수행하는 함수입니다. 구체적인 내용은 센서별 데이터 시트를 참고해 주세요.
  virtual bool parse();

  std::vector<std::string> paramName;
  std::vector<double> imuData;
  //remember that SensorConfigurator has a communicator member : _ptr
};
