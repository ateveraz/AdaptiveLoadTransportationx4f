classdef Modelx4 < matlab.System
    % Modelx4 Matlab system object for a x4 model dynamic. 
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    properties (Access = private)
        gravity = 9.81;
        e3 = [0 0 1]';
        quat;
    end

    properties (Access = public)
        mass = 0.4;
        inertia_matrix = [2.098e3 63.577538 -2.002648; 63.577538 2.102e3 0.286186; -2.002648 0.286186 4.068e3]*1e-6;
    end

    % Pre-computed constants or internal states
    % properties (Access = private)

    % end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.quat = Quaternions();
        end

        function [xpp, wp, quatp] = stepImpl(obj, tau, thrust, w, quat)     
            wp = obj.attitudeDynamics(tau, w);
            quatp = obj.w2qtp(quat, w);
            xpp = obj.cartesianDynamics(thrust, quat);
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end

        function [xpp, wp, quatp] = getOutputSizeImpl(~)
            xpp = [3 1];
            wp = [3 1];
            quatp = [4 1];
        end

        function [xpp, wp, quatp] = getOutputDataTypeImpl(~)
            xpp = 'double';
            wp = 'double';
            quatp = 'double';
        end

        function [xpp, wp, quatp] = isOutputComplexImpl(~)
            xpp = false;
            wp = false;
            quatp = false;
        end

        function [xpp, wp, quatp] = isOutputFixedSizeImpl(~)
            xpp = true;
            wp = true;
            quatp = true;
        end
    end

    methods (Access = private)
        function wp = attitudeDynamics(obj, tau, w)
            wp = obj.inertia_matrix\tau-obj.PCO(w)*w;
        end

        function qtp = w2qtp(obj, qt, w)
            w_qt = [0 w(1) w(2) w(3)]';
            qtp = (1/2)*obj.quat.product(qt, w_qt);
        end

        function xipp = cartesianDynamics(obj, thrust, qt)
            % Compute quaternion (qt) in body z-coordinate: qtb
            qtc = obj.quat.conj(qt);
            qtxe3 = obj.quat.product(qt,[0; obj.e3]);
            qtxe3xqtc = obj.quat.product(qtxe3,qtc);
            qtb = qtxe3xqtc(2:4); 

            % Cartesian Dynamics
            xipp = thrust*qtb/obj.mass - obj.gravity*obj.e3;
        end

        function S = PCO(~, w)
            S = [0 -w(3) w(2); 
                 w(3) 0 -w(1);
                -w(2) w(1) 0];
        end
    end
end
