import unittest
import numpy as np
from core import uncertainty_gain,directional_uncertainty_gain


class GainTest(unittest.TestCase):
    def test_halving_has_same_value_before_and_after_large_reduction(self):
        gains=uncertainty_gain(100.,np.array([100.,50.,2.,1.]),'log')
        self.assertAlmostEqual(gains[1]-gains[0],np.log(2.))
        self.assertAlmostEqual(gains[3]-gains[2],np.log(2.))
        # With a 0.1 movement penalty, the original score incorrectly hides
        # a 50% reduction once the variance is small relative to the baseline.
        old=uncertainty_gain(100.,np.array([2.,1.]))
        self.assertLess(old[1]-old[0],.1)
        self.assertGreater(gains[3]-gains[2],.1)

    def test_zero_evidence_and_scale_invariance(self):
        np.testing.assert_array_equal(uncertainty_gain(0.,[0.,0.],'log'),[0.,0.])
        self.assertEqual(uncertainty_gain(3.,[3.],'log')[0],0.)
        np.testing.assert_allclose(uncertainty_gain(100.,[50.,20.],'log'),
                                   uncertainty_gain(1.,[.5,.2],'log'))

    def test_old_mode_unchanged(self):
        p=np.array([3.,2.,1.,0.])
        np.testing.assert_array_equal(uncertainty_gain(3.,p),(3.-p)/3.)
        with self.assertRaises(ValueError):uncertainty_gain(3.,p,'other')

    def test_directional_probabilities_do_not_cancel(self):
        base=np.array([100.,10.,1.]);post=np.array([[10.,5.,1.],[50.,2.,.5]])
        weight=np.array([.8,.2,.1])
        value=directional_uncertainty_gain(base,post,weight)
        np.testing.assert_allclose(directional_uncertainty_gain(base,post,weight*.01),value*.01)
        np.testing.assert_array_equal(directional_uncertainty_gain(base,post,weight*0),[0.,0.])

    def test_directional_units_and_nonweak_directions(self):
        base=np.array([1.,100.]);post=np.array([[.1,10.],[.9,.01]])
        # Only the first weak direction counts, irrespective of the other
        # direction's units, absolute variance, or much larger improvement.
        value=directional_uncertainty_gain(base,post,[1.,0.])
        self.assertGreater(value[0],value[1])
        scale=np.array([1e-5,1e5])
        np.testing.assert_allclose(value,directional_uncertainty_gain(base*scale,post*scale,[1.,0.]))

    def test_directional_uses_coupled_covariance(self):
        H=np.array([[1.,.9],[.9,1.]])
        added=np.diag([0.,10.])
        before=np.diag(np.linalg.inv(H));after=np.diag(np.linalg.inv(H+added))
        # Measuring the second coordinate also reduces marginal uncertainty
        # along the first, even though H[0,0] itself is unchanged.
        self.assertGreater(directional_uncertainty_gain(before,after,[1.,0.]),1.)


if __name__=='__main__':unittest.main()
