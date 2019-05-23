/**
 * @file rosflight_api.cpp
 * @brief Entry point for ROSflight MATLAB MEX
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 23 May 2019
 */

#include <iostream>
#include <memory>
#include <stdint.h>
#include <string>
#include <cstring>
#include <typeinfo>

#include <mex.h>
#include <matrix.h>
#include "matlab_utils.h"

#include <rosflight.h>
#include <mavlink/mavlink.h>
#include <rosflight_matlab/matlab_board.h>

using namespace rosflight_firmware;

class ROSflightAPI
{
public:
  ROSflightAPI()
  {
    //
    // Initialize ROSflight autopilot
    //

    board_.reset(new rosflight_matlab::MATLABBoard);
    mavlink_.reset(new rosflight_firmware::Mavlink(*board_));
    firmware_.reset(new rosflight_firmware::ROSflight(*board_, *mavlink_));

    firmware_->init();
  }
  ~ROSflightAPI() = default;

  // --------------------------------------------------------------------------
  
  void setTime(const mxArray * _time) const
  {
    if (mxGetNumberOfElements(_time) != 1) {
      mexErrMsgTxt("Expected time data to be a scalar.");
      return;
    }

    board_->setTime(*mxGetDoubles(_time));
  }

  // --------------------------------------------------------------------------

  void setImu(const mxArray * _gyro, const mxArray * _accel) const
  {

    if (mxGetNumberOfElements(_gyro) != 3) {
      mexErrMsgTxt("Expected gyro data to have 3 elements.");
      return;
    }

    if (mxGetNumberOfElements(_accel) != 3) {
      mexErrMsgTxt("Expected accel data to have 3 elements.");
      return;
    }

    float gyro[3], accel[3];

    gyro[0] = static_cast<float>(mxGetDoubles(_gyro)[0]);
    gyro[1] = static_cast<float>(mxGetDoubles(_gyro)[1]);
    gyro[2] = static_cast<float>(mxGetDoubles(_gyro)[2]);

    accel[0] = static_cast<float>(mxGetDoubles(_accel)[0]);
    accel[1] = static_cast<float>(mxGetDoubles(_accel)[1]);
    accel[2] = static_cast<float>(mxGetDoubles(_accel)[2]);

    board_->setIMU(gyro, accel);
  }

  // --------------------------------------------------------------------------

  void run() const
  {
    firmware_->run();
  }

  // --------------------------------------------------------------------------

  void getState(mxArray * plhs[]) const
  {
    plhs[0] = mxCreateDoubleMatrix(1, 4, mxREAL); // quaternion
    plhs[1] = mxCreateDoubleMatrix(1, 3, mxREAL); // Euler RPY
    plhs[2] = mxCreateDoubleMatrix(1, 3, mxREAL); // omega
    plhs[3] = mxCreateDoubleMatrix(1, 1, mxREAL); // timestamp

    auto quat = firmware_->estimator_.state().attitude;
    mxGetDoubles(plhs[0])[0] = static_cast<double>(quat.w);
    mxGetDoubles(plhs[0])[1] = static_cast<double>(quat.x);
    mxGetDoubles(plhs[0])[2] = static_cast<double>(quat.y);
    mxGetDoubles(plhs[0])[3] = static_cast<double>(quat.z);

    float roll = firmware_->estimator_.state().roll;
    float pitch = firmware_->estimator_.state().pitch;
    float yaw = firmware_->estimator_.state().yaw;
    mxGetDoubles(plhs[1])[0] = static_cast<double>(roll);
    mxGetDoubles(plhs[1])[1] = static_cast<double>(pitch);
    mxGetDoubles(plhs[1])[2] = static_cast<double>(yaw);

    auto omega = firmware_->estimator_.state().angular_velocity;
    mxGetDoubles(plhs[2])[0] = static_cast<double>(omega.x);
    mxGetDoubles(plhs[2])[1] = static_cast<double>(omega.y);
    mxGetDoubles(plhs[2])[2] = static_cast<double>(omega.z);

    uint64_t time = firmware_->estimator_.state().timestamp_us;
    *mxGetDoubles(plhs[3]) = static_cast<double>(time*1e-6);
  }

  // --------------------------------------------------------------------------

  void setParam(const mxArray * _paramName, const mxArray * _value)
  {
    // TODO: The '+1' is a hack to deal with the fact that ROSflight param
    // names are initialized with null-terminated strings. GYROXY_LPF_ALPHA is
    // missing it's trailing A (from MATLAB) otherwise.
    char paramName[Params::PARAMS_NAME_LENGTH];
    mxGetString(_paramName, paramName, Params::PARAMS_NAME_LENGTH+1);

    // convert param name to id
    uint16_t paramId = firmware_->params_.lookup_param_id(paramName);

    // determine the param type and then use that method
    auto paramType = firmware_->params_.get_param_type(paramId);
    if (paramType == PARAM_TYPE_INT32) {
      uint32_t value = static_cast<uint32_t>(*mxGetDoubles(_value));
      firmware_->params_.set_param_int(paramId, value);

    } else if (paramType == PARAM_TYPE_FLOAT) {
      float value = static_cast<float>(*mxGetDoubles(_value));
      firmware_->params_.set_param_float(paramId, value);

    } else {
      mexErrMsgTxt("Unknown paramId.");
      return;
    }
  }

  // --------------------------------------------------------------------------

  void getParam(const mxArray * _paramName, mxArray * &_value)
  {
    char paramName[Params::PARAMS_NAME_LENGTH];
    mxGetString(_paramName, paramName, Params::PARAMS_NAME_LENGTH);

    // convert param name to id
    uint16_t paramId = firmware_->params_.lookup_param_id(paramName);

    // create space for the return value
    _value = mxCreateDoubleMatrix(1, 1, mxREAL);

    // determine the param type and then use that method
    auto paramType = firmware_->params_.get_param_type(paramId);
    if (paramType == PARAM_TYPE_INT32) {
      int value = firmware_->params_.get_param_int(paramId);
      *mxGetDoubles(_value) = static_cast<double>(value);

    } else if (paramType == PARAM_TYPE_FLOAT) {
      float value = firmware_->params_.get_param_float(paramId);
      std::cout << "Setting value: " << value << std::endl;
      *mxGetDoubles(_value) = static_cast<double>(value);

    } else {
      mexErrMsgTxt("Unknown paramId.");
      return;
    }
  }
  
private:
  // ROSflight objects
  std::unique_ptr<rosflight_matlab::MATLABBoard> board_;
  std::unique_ptr<rosflight_firmware::Mavlink> mavlink_;
  std::unique_ptr<rosflight_firmware::ROSflight> firmware_;
};

// ============================================================================
// ============================================================================

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // nlhs   number of expected outputs
  // plhs   array to be populated by outputs (data passed back to matlab)
  // nrhs   number of inputs
  // prhs   array poplulated by inputs (data passed from matlab)

  // forward std::cout to MATLAB console
  mxstreambuf mout;

  //
  // Input/Output Checking
  //

  // prhs[0] (the first input) is always the command string
  // mxGetString (from mex.h) converts prhs[0] to char array
  char cmd[64];
  if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd))) {
    mexErrMsgTxt("first input should be a command string less than 64 characters long.");
    return;
  }

  // if not requesting new, two inputs are expected (operation and handle)
  if (!!strcmp("new", cmd) && nrhs < 2) {
    mexErrMsgTxt("second input should be a class instance handle.");
    return;
  }

  //
  // MATLAB Object Lifecycle
  //

  // is the input from matlab requesting new object?
  if (!strcmp("new", cmd) && nlhs == 1) {
    plhs[0] = convertPtr2Mat<ROSflightAPI>(new ROSflightAPI);
    return;
  }

  // input from matlab requesting object deletion?
  if (!strcmp("delete", cmd) && nlhs == 0 && nrhs == 2) {
    destroyObject<ROSflightAPI>(prhs[1]);
    return;
  }

  // for convenience
  auto rfapi = convertMat2Ptr<ROSflightAPI>(prhs[1]);
  size_t args = nrhs-2; // args: [cmd, handle, ...]

  //
  // ROSflight API Bindings
  //
  
  if (!strcmp("set_imu", cmd) && nlhs == 0 && args == 2) {
    rfapi->setImu(prhs[2], prhs[3]);
    return;
  }

  if (!strcmp("set_time", cmd) && nlhs == 0 && args == 1) {
    rfapi->setTime(prhs[2]);
    return;
  }

  if (!strcmp("run", cmd) && nlhs == 0 && args == 0) {
    rfapi->run();
    return;
  }

  if (!strcmp("get_state", cmd) && nlhs == 4 && args == 0) {
    rfapi->getState(plhs);
    return;
  }

  if (!strcmp("set_param", cmd) && nlhs == 0 && args == 2) {
    rfapi->setParam(prhs[2], prhs[3]);
    return;
  }

  if (!strcmp("get_param", cmd) && nlhs == 1 && args == 1) {
    rfapi->getParam(prhs[2], plhs[0]);
    return;
  }

  mexErrMsgTxt("Unrecognized ROSflight API command.");

}
