classdef PositionController < matlab.System
    % Controller Matlab system object for an attitude controller.

    properties (Access = public)
        Kp = 1.0;
        Kd = 2.0;
        Ki = 0.5;
        mass = 0.4;
    end

    properties (Access = private)
        gravity = [0 0 9.81]';
    end

    methods (Access = protected)
        function setupImpl(~)
        end

        function [u, up] = stepImpl(obj, e, ep, int_e)
            u = - obj.Kp * e - obj.Kd * ep - obj.Ki * int_e + obj.mass * obj.gravity;
            up = - obj.Kp * ep - obj.Ki * e;
        end

        function resetImpl(~)
            % Initialize / reset internal properties
        end

        function [u, up] = getOutputSizeImpl(~)
            u = [3 1];
            up = [3 1];
        end

        function [u, up] = getOutputDataTypeImpl(~)
            u = 'double';
            up = 'double';
        end

        function [u, up] = isOutputComplexImpl(~)
            u = false;
            up = false;
        end

        function [u, up] = isOutputFixedSizeImpl(~)
            u = true;
            up = true;
        end
    end
end
