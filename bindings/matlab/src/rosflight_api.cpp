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
  }
  ~ROSflightAPI() = default;

  // --------------------------------------------------------------------------
  
  void setTime(const mxArray * _time) const
  {
    if (mxGetNumberOfElements(_time) != 1) {
      mexErrMsgTxt("Expected time data to be a scalar.");
    }

    board_->setTime(*mxGetDoubles(_time));
  }

  // --------------------------------------------------------------------------

  void setImu(const mxArray * _gyro, const mxArray * _accel) const
  {

    if (mxGetNumberOfElements(_gyro) != 3) {
      mexErrMsgTxt("Expected gyro data to have 3 elements.");
    }

    if (mxGetNumberOfElements(_accel) != 3) {
      mexErrMsgTxt("Expected accel data to have 3 elements.");
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

  void run() const { firmware_->run(); }
  
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

}
