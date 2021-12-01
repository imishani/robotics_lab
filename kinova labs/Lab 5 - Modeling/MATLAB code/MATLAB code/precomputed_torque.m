function [couple] = precomputed_torque(q,qp,qpp)
couple=inertia(q)*qpp'+coriolis(q,qp)*qp'+gravity(q)';
end