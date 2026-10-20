"""Position-uncertainty reduction, retaining rotation/translation coupling.

This is a planning score, not a calibrated forecast of the filter's accuracy.
The first three tangent coordinates are rotation; the last three are position
at the chosen body origin. Rotation is marginalized, not assumed exact.
"""
import numpy as np


def translation_gain(prior, contributions, weights):
    prior = (prior+prior.T)*.5
    if prior.shape != (6, 6):
        raise ValueError('Six-dimensional pose information required')
    additions = np.einsum('cn,nij->cij', weights, contributions)
    posterior = prior[None]+additions
    selector = np.vstack([np.zeros((3, 3)), np.eye(3)])
    def covariance_logdet(information):
        L = np.linalg.cholesky((information+np.swapaxes(information,-1,-2))*.5)
        rhs = np.broadcast_to(selector, L.shape[:-2]+selector.shape)
        projected = np.linalg.solve(L, rhs)
        covariance = np.swapaxes(projected,-1,-2)@projected
        C = np.linalg.cholesky((covariance+np.swapaxes(covariance,-1,-2))*.5)
        return 2*np.log(np.diagonal(C,axis1=-2,axis2=-1)).sum(axis=-1)
    return covariance_logdet(prior)-covariance_logdet(posterior)
