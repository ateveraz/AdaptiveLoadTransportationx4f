classdef DesiredAttitude < matlab.System
    % DesiredAttitude 

    properties (Access = private)
        quat;
        Fb = [0 0 0 1]';
    end


    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.quat = Quaternions();
        end

        function [qe, we] = stepImpl(obj, fu, w, q)
            fu_norm = fu / norm(fu);
            quatF = obj.quat.product([0 ; fu_norm], obj.quat.conj(obj.Fb));
            qd = obj.quat.exp(obj.quat.log(quatF)/2);

            qe = obj.computeQuaternionError(qd, q);

            wd = [0 0 0]';

            we = w - wd;
        end

        function resetImpl(~)
            % Initialize / reset internal properties
        end

        function [qd, wd] = getOutputSizeImpl(~)
            qd = [4 1];
            wd = [3 1];
        end

        function [qd, wd] = getOutputDataTypeImpl(~)
            qd = 'double';
            wd = 'double';
        end

        function [qd, wd] = isOutputComplexImpl(~)
            qd = false;
            wd = false;
        end

        function [qd, wd] = isOutputFixedSizeImpl(~)
            qd = true;
            wd = true;
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