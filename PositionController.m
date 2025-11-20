classdef PositionController < matlab.System
    % PositionController Controlador de posición con RBFNN

    properties (Access = public)
        Kd = 2.0;
        alpha = 1.0;
        mass = 0.4;
        Gamma = 0.1;
        kappa = 0.01;
        k = 10;
    
        N_neurons = 10;
        center_range = 1.0;
        eta_width = 1.5;
    end

    properties (Access = private)
        gravity = [0 0 9.81]';
        nn; 
        dim_in  = 6; 
        dim_out = 3;
    end

    methods (Access = protected)
        function setupImpl(obj)
            mu = (rand(obj.dim_in, obj.N_neurons) * 2 - 1) * obj.center_range;
            eta = obj.eta_width;
            obj.nn = RBFNN(mu, eta, obj.Gamma, obj.kappa, obj.dim_out);
        end

        function [u, rbf, up, wp] = stepImpl(obj, t, xLdpp, Td, e, ep, w) 
            
            s = ep + obj.alpha * e;
            x = [e; ep];

            [Kdhat, wp] = obj.nn.compute_update(s, w, x);

            %learningPhase = 1 - exp(-obj.k*t);
            t0 = 5/obj.k;
            learningPhase = 1 ./ (1 + exp(-obj.k * (t - t0)));
            rbf = learningPhase * Kdhat .* s ;

            % u_fl = obj.mass * (obj.gravity + xLdpp - obj.alpha*ep + Td);
            
            % u = u_fl - obj.Kd * s;

            nui = - obj.Kd * s;
            u = obj.mass * (obj.gravity + xLdpp) - Td + nui; % PD Control

            up = zeros(3,1); % Fix it ? 
        end

        function resetImpl(~)
        end

        function [u, rbf, up, wp] = getOutputSizeImpl(obj)
            u  = [obj.dim_out 1];
            up = [obj.dim_out 1];
            rbf = [obj.dim_out 1];
            wp = [obj.dim_out obj.N_neurons];
        end

        function [u, rbf, up, wp] = getOutputDataTypeImpl(~)
            u = 'double';
            up = 'double';
            rbf = 'double';
            wp = 'double';
        end

        function [u, rbf, up, wp] = isOutputComplexImpl(~)
            u = false;
            up = false;
            rbf = false;
            wp = false;
        end

        function [u, rbf, up, wp] = isOutputFixedSizeImpl(~)
            u = true;
            up = true;
            rbf = true;
            wp = true;
        end
    end
end