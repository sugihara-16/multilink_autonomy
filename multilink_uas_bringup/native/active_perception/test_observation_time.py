import unittest
import numpy as np
from observation_time import latest_not_after

class TimeAssociation(unittest.TestCase):
    def test_new_callback_does_not_hide_usable_past(self):
        old=(1_000_000_000,'old');new=(1_050_000_000,'new')
        self.assertEqual(latest_not_after([old,new],1.02),old)
    def test_future_only_and_stale_are_rejected(self):
        self.assertIsNone(latest_not_after([(1_050_000_000,)],1.02))
        self.assertIsNone(latest_not_after([(1_000_000_000,)],1.6))
    def test_reordered_delivery_uses_measurement_order(self):
        self.assertEqual(latest_not_after([(1_000_000_000,),(900_000_000,)],1.1)[0],1_000_000_000)
    def test_nan_or_infinite_or_empty_rejected(self):
        for records in [[],[(float('nan'),)],[(float('inf'),)]]:
            self.assertIsNone(latest_not_after(records,1.))
    def test_random_asynchronous_arrivals(self):
        rng=np.random.default_rng(4771)
        for _ in range(1000):
            times=np.arange(20)*.05;ref=float(rng.uniform(.05,.95));rng.shuffle(times)
            record=latest_not_after([(round(t*1e9),) for t in times],ref)
            self.assertGreaterEqual(ref-record[0]*1e-9,0)
            self.assertLess(ref-record[0]*1e-9,.05+1e-9)

if __name__=='__main__':unittest.main()
