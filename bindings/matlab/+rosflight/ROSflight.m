% The main algorithm interface to the C++ ROSflight.

classdef ROSflight < handle
    properties (SetAccess = private, Hidden = true)
        % Handles to underlying C++ class instances
        hBoard;
        hMavlink;
        hRF;
    end
    methods
        % constructor
        % the 'new' tells the mex function to instantiate the rransac class
        % the mex function will also call mexLock() which will keep the object
        % in memory and will do a std::reinterpret_cast on the object pointer
        % in order to pass the pointer back to the API, which is assigned as
        % this.objectHandle
        function this = ROSflight()
            
            % instantiate the C++ class using the parameters
            [this.hBoard, this.hMavlink, this.hRF] = rosflight.rosflight('new');
        end

        % destructor
        % Tells the mex function to destroy the C++ class instance
        function delete(this)
            rosflight.rosflight('delete', this.hBoard, this.hMavlink, this.hRF);
        end

    end
end