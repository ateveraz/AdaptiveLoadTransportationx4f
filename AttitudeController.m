classdef AttitudeController < matlab.System
    % Controller Matlab system object for an attitude controller.

    properties (Access = public)
        Kd = 2.0;
        beta = 0.5;
        gamma = 1.0;
        rho = 5.0;
        mass = 0.4;
        gravity = [0 0 9.81]';
        Fb = [0 0 0 1]';
        e3 = [0 0 1]';
    end

    % Pre-computed constants or internal states
    % properties (Access = private)

    % end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.quat = Quaternions();
        end

        function [tau, thrust] = stepImpl(obj, u, qe, we)
            s = we + obj.rho * qe(2:4);
            tau = - obj.Kd*s - obj.beta * tanh( obj.gamma * s);
            thrust = norm(u);
        end

        function resetImpl(~)
            % Initialize / reset internal properties
        end

        function [tau, thrust] = getOutputSizeImpl(~)
            tau = [3 1];
            thrust = [1 1];
        end

        function [tau, thrust] = getOutputDataTypeImpl(~)
            tau = 'double';
            thrust = 'double';
        end

        function [tau, thrust] = isOutputComplexImpl(~)
            tau = false;
            thrust = false;
        end

        function [tau, thrust] = isOutputFixedSizeImpl(~)
            tau = true;
            thrust = true;
        end
    end
end
