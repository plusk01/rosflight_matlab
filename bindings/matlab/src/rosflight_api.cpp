#include <iostream>
#include <stdint.h>
#include <string>
#include <cstring>
#include <typeinfo>

#include <mex.h>
#include "matlab_utils.h"

#include <rosflight.h>
#include <mavlink/mavlink.h>
#include <rosflight_matlab/matlab_board.h>

// ----------------------------------------------------------------------------

void mexNew(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // check parameters
  if (nlhs != 3) {
    mexErrMsgTxt("mexNew: three outputs expected.");
    return;
  }

  auto board = new rosflight_matlab::MATLABBoard;
  auto mavlink = new rosflight_firmware::Mavlink(*board);
  auto rf = new rosflight_firmware::ROSflight(*board, *mavlink);

  plhs[0] = convertPtr2Mat<rosflight_matlab::MATLABBoard>(board);
  plhs[1] = convertPtr2Mat<rosflight_firmware::Mavlink>(mavlink);
  plhs[2] = convertPtr2Mat<rosflight_firmware::ROSflight>(rf);
}

// ----------------------------------------------------------------------------

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
  }

  // if not requesting new, two inputs are expected (operation and handle)
  if (!!strcmp("new", cmd) && nrhs < 2) {
      mexErrMsgTxt("second input should be a class instance handle.");
  }

  //
  // MATLAB requests a new rosflight object
  //

  // is the input from matlab requesting new object?
  if (!strcmp("new", cmd)) {
      mexNew(nlhs, plhs, nrhs, prhs);
      return;
  }

}
