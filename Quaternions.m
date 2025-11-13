classdef Quaternions
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here

    properties
    end

    methods (Access = public)
        function obj = Quaternions()
        end

        function qc = conj(~, q)
            qc = q;
            qc(2:4) = - q(2:4);
        end

        function qR = product(~, p, q)
            qR = zeros(4,1);
            qR(1) = p(1)*q(1) - dot(p(2:4),q(2:4));
            qR(2:4) = p(1)*q(2:4) + q(1)*p(2:4) + cross(p(2:4),q(2:4));
        end

        function logQ = log(~,q_e)
            w = q_e(1);
            v = q_e(2:4);
            norm_v = norm(v);
            epsilon = 1e-9; 
            
            if (norm_v < epsilon)
                v_prime = [0, 0, 0]';
            else
                if (w > 1.0) w = 1.0; end
                if (w < -1.0) w = -1.0; end
                theta_half = acos(w);
                v_prime = v * (theta_half / norm_v);
            end
            logQ = v_prime';
        end

        function q_out = exp(~, v_prime)
            
            L = norm(v_prime); 

            if (L == 0)
                w = 1.0;
                v = [0, 0, 0];
            else
                w = cos(L);
                scale = sin(L) / L;
                v = v_prime * scale;
            end
            q_out = [w, v(1), v(2), v(3)]';
        end
    end
end