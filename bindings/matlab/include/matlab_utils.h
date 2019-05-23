#pragma once

#include <stdint.h>
#include <string>
#include <cstring>
#include <typeinfo>

#include <mex.h>

#define CLASS_HANDLE_SIGNATURE 0xFF00F0A5

// This is the rosflight api that is compiled into rosflight.mexa64. The
// rosflight matlab class found in rosflight.m always calls this looking for the
// function named "mexFunction". The arguments passed determine the operations
// performed at the c++ level including class intantiation, desctruction, and
// the calling of class methods.

// ----------------------------------------------------------------------------

template<class base> class class_handle
{
public:
    class_handle(base *ptr) : ptr_m(ptr), name_m(typeid(base).name()) { signature_m = CLASS_HANDLE_SIGNATURE; }
    ~class_handle() { signature_m = 0; delete ptr_m; }
    bool isValid() { return ((signature_m == CLASS_HANDLE_SIGNATURE) && !strcmp(name_m.c_str(), typeid(base).name())); }
    base *ptr() { return ptr_m; }

private:
    uint32_t signature_m;
    std::string name_m;
    base *ptr_m;
};

template<class base> inline mxArray *convertPtr2Mat(base *ptr)
{
    mexLock();
    mxArray *out = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    *((uint64_t *)mxGetData(out)) = reinterpret_cast<uint64_t>(new class_handle<base>(ptr));
    return out;
}

template<class base> inline class_handle<base> *convertMat2HandlePtr(const mxArray *in)
{
    if (mxGetNumberOfElements(in) != 1 || mxGetClassID(in) != mxUINT64_CLASS || mxIsComplex(in))
        mexErrMsgTxt("input must be a real uint64 scalar.");
    class_handle<base> *ptr = reinterpret_cast<class_handle<base> *>(*((uint64_t *)mxGetData(in)));
    if (!ptr->isValid())
        mexErrMsgTxt("handle not valid.");
    return ptr;
}

template<class base> inline base *convertMat2Ptr(const mxArray *in)
{
    return convertMat2HandlePtr<base>(in)->ptr();
}

template<class base> inline void destroyObject(const mxArray *in)
{
    delete convertMat2HandlePtr<base>(in);
    mexUnlock();
}

// ----------------------------------------------------------------------------

// The following class redirects stdout to use mexPrintf so that std::cout
// appears in the MATLAB console. Having this class in the code is enough
// to allow any std::cout calls in any linked object to be redirected.
//
// To use in std::cout in this MEX-C++ code, you must instantiate an
// object first, as in the following example:
//
//      mxstreambuf mout;
//      std::cout << "Hi!" << std::endl;
//
// See https://stackoverflow.com/a/41276477/2392520
class mxstreambuf : public std::streambuf {
   public:
      mxstreambuf() {
         stdoutbuf = std::cout.rdbuf( this );
      }
      ~mxstreambuf() {
         std::cout.rdbuf( stdoutbuf );
      }
   protected:
      virtual std::streamsize xsputn( const char* s, std::streamsize n ) override {
         mexPrintf( "%.*s", n, s );
         return n;
      }
      virtual int overflow( int c = EOF ) override {
         if( c != EOF ) {
            mexPrintf( "%.1s", & c );
         }
         return 1;
      }
   private:
      std::streambuf *stdoutbuf;
};
