% bindings to static methods and utilities

classdef utils

    methods(Static)

        function v = boxminus(q1, q2)
            v = rosflight.rosflight_api('utils_boxminus', 0, q1, q2);
        end

    end

end