import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from translation_information_gain import translation_gain
from joint_information_view import joint_gain


class TranslationInformationTest(unittest.TestCase):
    def test_prefers_position_to_redundant_rotation_information(self):
        parts=np.zeros((2,6,6));parts[0,0,0]=100;parts[1,3,3]=10
        self.assertGreater(joint_gain(np.eye(6),parts,np.eye(2))[0],joint_gain(np.eye(6),parts,np.eye(2))[1])
        score=translation_gain(np.eye(6),parts,np.eye(2))
        np.testing.assert_allclose(score,[0,np.log(11)],atol=1e-12)

    def test_orientation_observation_can_reduce_correlated_position_error(self):
        prior=np.eye(6);prior[0,3]=prior[3,0]=.95
        part=np.zeros((1,6,6));part[0,0,0]=1
        score=translation_gain(prior,part,np.ones((1,1)))
        expected=np.log((1/(1-.95**2))/(1/(1-.95**2/2)))
        np.testing.assert_allclose(score,[expected],atol=1e-12)
        self.assertGreater(score[0],0)

    def test_matches_explicit_covariance_marginalization(self):
        rng=np.random.default_rng(23)
        for unused in range(30):
            A=rng.normal(size=(6,6));H=A.T@A+.1*np.eye(6)
            J=rng.normal(size=(8,2,6));parts=J.transpose(0,2,1)@J
            weights=rng.uniform(size=(5,8));post=H+np.einsum('cn,nij->cij',weights,parts)
            reference=np.linalg.slogdet(np.linalg.inv(H)[3:,3:])[1]-np.linalg.slogdet(np.linalg.inv(post)[:,3:,3:])[1]
            np.testing.assert_allclose(translation_gain(H,parts,weights),reference,atol=1e-11)

    def test_coordinate_rotation_and_units_preserve_ranking(self):
        rng=np.random.default_rng(42);A=rng.normal(size=(6,6));H=A.T@A+np.eye(6)
        J=rng.normal(size=(4,2,6));parts=J.transpose(0,2,1)@J;weights=rng.uniform(size=(3,4))
        S=np.zeros((6,6));S[:3,:3]=Rotation.from_rotvec([.2,.6,-.1]).as_matrix()*.1
        S[3:,3:]=Rotation.from_rotvec([.4,-.2,.3]).as_matrix()*100
        np.testing.assert_allclose(translation_gain(H,parts,weights),translation_gain(S.T@H@S,S.T@parts@S,weights),atol=1e-11)

    def test_zero_measurement_gives_zero_gain(self):
        np.testing.assert_allclose(translation_gain(np.eye(6),np.zeros((1,6,6)),np.ones((3,1))),0,atol=1e-12)

    def test_rejects_singular_prior(self):
        with self.assertRaises(np.linalg.LinAlgError):
            translation_gain(np.zeros((6,6)),np.ones((1,6,6)),np.ones((1,1)))


if __name__=='__main__':
    unittest.main()
