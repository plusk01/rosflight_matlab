% The main algorithm interface to the C++ ROSflight.

classdef ROSflight < handle
    properties (SetAccess = private, Hidden = true)
        hObj; % Handle to underlying C++ class instance

        % rotation of a FRD body w.r.t a FLU body (rotates FRD into FLU)
        R_flu_frd = [1 0 0; 0 -1 0; 0 0 -1];
    end
    properties (SetAccess = public, Hidden = false)
        frame = 'FRD';
    end
    methods
        % constructor
        % the 'new' tells the mex function to instantiate a ROSflightAPI obj.
        % The mex function will also call mexLock() which will keep the object
        % in memory and will do a std::reinterpret_cast on the object pointer
        % in order to pass the pointer back to the API, which is assigned as
        % this.objectHandle
        function this = ROSflight()
            
            % instantiate the C++ class using the parameters
            this.hObj = rosflight.rosflight_api('new');
        end

        % destructor
        % Tells the mex function to destroy the C++ class instance
        function delete(this)
            rosflight.rosflight_api('delete', this.hObj);
        end

        function setIMU(this, gyro, accel)
            %SETIMU
            gyro = this.rotIn(gyro);
            accel = this.rotIn(accel);
            rosflight.rosflight_api('set_imu', this.hObj, gyro, accel);
        end

        function setTime(this, time)
            %SETTIME
            rosflight.rosflight_api('set_time', this.hObj, time);
        end

        function run(this)
            %RUN
            rosflight.rosflight_api('run', this.hObj);
        end

        function [q, rpy, omega, t] = getState(this)
            %GETSTATE
            [q, rpy, omega, t] = rosflight.rosflight_api('get_state', this.hObj);

            q(2:4) = this.rotOut(q(2:4));
            rpy = this.rotOut(rpy);
            omega = this.rotOut(omega);
        end

        function setParam(this, paramName, value)
            rosflight.rosflight_api('set_param', this.hObj, paramName, value);
        end

        function value = getParam(this, paramName)
            value = rosflight.rosflight_api('get_param', this.hObj, paramName);
        end

    end

    methods (Access = private)
        function v = rotIn(this, v)
            %ROTIN ROSflight uses FRD. Make the incoming data be FRD.

            oldshape = size(v);
            v = reshape(v, 3, 1);

            if strcmp(this.frame, 'FRD')
              v = v; % data is already in FRD
            elseif strcmp(this.frame, 'FLU')
              v = this.R_flu_frd'*v;
            else
                error(['Frame ''' this.frame ''' not defined']);
            end

            v = reshape(v, oldshape);
        end

        function v = rotOut(this, v)
            %ROTIN ROSflight uses FRD. Make the outgoing data fit the user's.

            oldshape = size(v);
            v = reshape(v, 3, 1);

            if strcmp(this.frame, 'FRD')
              v = v; % data is already in FRD
            elseif strcmp(this.frame, 'FLU')
              v = this.R_flu_frd*v;
            else
                error(['Frame ''' this.frame ''' not defined']);
            end

            v = reshape(v, oldshape);
        end
    end
end