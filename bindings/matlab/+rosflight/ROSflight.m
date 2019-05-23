% The main algorithm interface to the C++ ROSflight.

classdef ROSflight < handle
    properties (SetAccess = private, Hidden = true)
        hObj; % Handle to underlying C++ class instance
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
            rosflight.rosflight_api('set_imu', this.hObj, gyro, accel);
        end

        function setTime(this, time)
            %SETTIME
            rosflight.rosflight_api('set_time', this.hObj, time);
        end

        function getState(this)
            %GETSTATE
            rosflight.rosflight_api('get_state', this.hObj);
        end

    end
end