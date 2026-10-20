"""Conditional velocity covariance reduction from correlated bearing observations.

This is a local planning proxy for another observation of tracked landmarks,
not a forecast of the future filter covariance. The joint covariance must use
the same body velocity and two-dimensional bearing tangent coordinates as the
measurement Jacobian. Unknown/new features are not assigned information.
"""
import numpy as np


def conditional_velocity_covariance(joint, visibility, bearing_sigma):
    """Return one 3x3 velocity covariance per candidate visibility vector.

    ``joint`` orders body velocity first, then two bearing coordinates per
    landmark. Cross-landmark and velocity/landmark covariances are preserved.
    Visibility scales measurement information, with zero excluding a feature.
    """
    C = np.asarray(joint, dtype=float)
    w = np.asarray(visibility, dtype=float)
    if w.ndim != 2 or C.shape != (3+2*w.shape[1],)*2:
        raise ValueError('Incompatible joint covariance and visibility shapes')
    if (not np.isfinite(C).all() or not np.isfinite(w).all()
            or not np.isfinite(bearing_sigma) or bearing_sigma <= 0
            or np.any(w < 0) or np.any(w > 1)):
        raise ValueError('Finite covariance, visibility in [0,1], positive noise required')
    if not np.allclose(C, C.T, rtol=1e-10, atol=1e-12):
        raise ValueError('Joint covariance must be symmetric')
    C = (C+C.T)*.5
    if np.linalg.eigvalsh(C).min() < -1e-10*max(1., np.linalg.norm(C, 2)):
        raise ValueError('Joint covariance must be positive semidefinite')
    np.linalg.cholesky(C[:3, :3])
    if w.shape[1] == 0:
        return np.broadcast_to(C[:3, :3], (len(w), 3, 3)).copy()
    W = np.repeat(np.sqrt(w), 2, axis=1)/bearing_sigma
    S = np.eye(C.shape[0]-3)[None] + W[:, :, None]*C[None, 3:, 3:]*W[:, None, :]
    B = C[None, :3, 3:]*W[:, None, :]
    L = np.linalg.cholesky(S)
    whitened = np.linalg.solve(L, B.transpose(0, 2, 1))
    posterior = C[None, :3, :3]-whitened.transpose(0, 2, 1)@whitened
    posterior = (posterior+posterior.transpose(0, 2, 1))*.5
    # Do not conceal an inconsistent covariance by clipping its eigenvalues.
    np.linalg.cholesky(posterior)
    return posterior


def velocity_gain(joint, visibility, bearing_sigma):
    posterior = conditional_velocity_covariance(joint, visibility, bearing_sigma)
    prior_logdet = np.linalg.slogdet(np.asarray(joint)[:3, :3])[1]
    return prior_logdet-np.linalg.slogdet(posterior)[1]
