classdef Controller < matlab.System
    % Controller Matlab system object for an attitude controller.

    properties (Access = private)
        quat;
    end

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

        function [tau, thrust] = stepImpl(obj, quat, w, u, xp)
            fu = u + obj.mass * obj.gravity;

            fu_norm = fu / norm(fu);
            quatF = obj.quat.product([0 ; fu_norm], obj.quat.conj(obj.Fb));
            qd = obj.quat.exp(obj.quat.log(quatF)/2);

            quat_error = obj.computeQuaternionError(qd, quat);

            s = w + obj.rho * quat_error(2:4);
            tau = - obj.Kd*s - obj.beta * tanh( obj.gamma * s);
            % logQ = 2 * obj.quat.log(quat_error)';
            % tau = -obj.Kp*logQ - obj.Kd*w;
            thrust = norm(fu);
        end

        function resetImpl(obj)
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

    methods (Access = private)
        function qe = computeQuaternionError(obj, qd, q)
            qd_conj = obj.quat.conj(qd);
            qe = obj.quat.product(qd_conj, q);
        end

        function qd = desiredQuaternion(obj, fu)
            qd = [0 0 0 0]';
            qd(1) = dot(obj.e3, fu);
            qd(2:4) = cross(obj.e3, fu);
            qd = qd / norm(qd);
        end
    end
end
