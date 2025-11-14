classdef RBFNN
    %RBFNN Red Neuronal de Función de Base Radial (Corregida)
    %   Aproxima una función f(z): R^m -> R^k
    %   En este caso, z es 6x1 (estado de error) y la salida es 3x1 (fuerza)

    properties
        mu    % Centros (Matriz [dim_in x N])
        eta   % Anchos (Vector [N x 1])
        Gamma % Tasa de Aprendizaje (Matriz 3x3 diagonal o escalar)
        kappa % Factor de amortiguamiento para aprendizaje robusto (escalar)
        
        N       % Número de neuronas (escalar)
        dim_in  % Dimensión de la entrada (escalar, ej. 6)
        dim_out % Dimensión de la salida (escalar, ej. 3)
    end

    methods (Access = public)
        function obj = RBFNN(mu, eta, Gamma, kappa, dim_out)
            obj.mu = mu;
            [obj.dim_in, obj.N] = size(mu);
            obj.dim_out = dim_out;
            
            if isscalar(eta)
                obj.eta = repmat(eta, obj.N, 1);
            else
                assert(length(eta) == obj.N, 'Eta should have N-elements');
                obj.eta = eta;
            end
            
            obj.Gamma = Gamma;
            obj.kappa = kappa;
        end

        function [nn_output, W_dot] = compute_update(obj, s, W, z)
            sigma_vec = obj.compute_sigma(z);
            nn_output = W * sigma_vec;
            W_dot = obj.Gamma * (abs(s) * sigma_vec' - obj.kappa * norm(s) * W);
        end

        function sigma_vec = compute_sigma(obj, z)
            sigma_vec = zeros(obj.N, 1);
            z_col = z(:); 
            
            for j = 1:obj.N
                mu_j = obj.mu(:, j);
                eta_j = obj.eta(j);
                dist_sq = sum((z_col - mu_j).^2);
                sigma_vec(j) = exp(-dist_sq / (2 * eta_j^2));
            end
        end
    end
end