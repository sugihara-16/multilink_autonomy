"""Track retention scored using ROVIO's joint velocity/bearing covariance."""
import numpy as np
from joint_information_view import JointInformationView
from joint_velocity_gain import velocity_gain


def decode(data):
    data = np.asarray(data, dtype=float)
    if data.ndim != 1 or len(data) < 3 or not np.isfinite(data).all() or data[0] != 1:
        raise ValueError('Invalid joint information message')
    stamp, count = data[1:3]
    if count != round(count) or not 0 <= count <= 25 or stamp != round(stamp) or not 0 <= stamp < 2**53:
        raise ValueError('Invalid joint information count/time')
    n = int(count); size = 3+2*n
    if len(data) != 3+4*n+size**2:
        raise ValueError('Joint information size mismatch')
    tracks = data[3:3+4*n].reshape(n, 4)
    if not np.array_equal(tracks[:, 0], np.rint(tracks[:, 0])) or len(set(tracks[:, 0])) != n:
        raise ValueError('Invalid/duplicate feature identity')
    C = data[3+4*n:].reshape(size, size).copy()
    return int(stamp), tracks[:, 0].astype(np.int64), tracks[:, 1:].copy(), C


def subset(ids, C, selected, rotation=None):
    indices = {int(ident): i for i, ident in enumerate(ids)}
    if len(indices) != len(ids):
        raise ValueError('Duplicate feature identity')
    try:
        columns = [0, 1, 2]+[3+2*indices[int(i)]+j for i in selected for j in range(2)]
    except KeyError as exc:
        raise ValueError('Covariance does not contain the observed feature') from exc
    result = C[np.ix_(columns, columns)].copy()
    if rotation is not None:
        R = np.asarray(rotation)
        if R.shape != (3, 3) or not np.allclose(R.T@R, np.eye(3), atol=1e-10) or np.linalg.det(R) < 0:
            raise ValueError('Invalid body-frame rotation')
        A = np.eye(len(result)); A[:3, :3] = R.T
        result = A@result@A.T
    return result


class JointVelocityView(JointInformationView):
    def set_joint_information(self, ids, covariance):
        self.joint_ids = np.asarray(ids).copy()
        self.joint_covariance = np.asarray(covariance).copy()

    def score_model(self, prior, contributions, ids):
        if not hasattr(self, 'joint_covariance'):
            raise ValueError('Missing matching ROVIO joint covariance')
        C = subset(self.joint_ids, self.joint_covariance, ids)
        return (lambda weights: velocity_gain(C, weights, self.cfg['bearing_sigma']),
                'conditional_joint_rovio_velocity_log_determinant_gain')
