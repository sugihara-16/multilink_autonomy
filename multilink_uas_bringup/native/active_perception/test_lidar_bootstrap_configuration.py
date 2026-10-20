import tempfile,unittest
from pathlib import Path
import yaml
from lidar_bootstrap_configuration import parameters


class ConfigurationTest(unittest.TestCase):
    def test_same_sensor_information_in_both_arms(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'camera.yaml';camera={'camera_matrix':{'data':[240.,0,240,0,240,135,0,0,1]},'distortion_coefficients':{'data':[0.]*5}}
            p.write_text(yaml.safe_dump(camera));imu={'gravity_magnitude':9.8,'acc_bias_tau':[175.]*3}
            d=parameters(imu,p,[-.15,0,.05,0,0,0,1],folder)
            a,b=d['rovio_gimbal'].copy(),d['fixed_rovio'].copy()
            self.assertNotEqual(a.pop('lidar_bootstrap_csv'),b.pop('lidar_bootstrap_csv'));self.assertEqual(a,b)
            self.assertNotIn('lidar_depth_bootstrap',imu)
            camera['distortion_coefficients']['data'][0]=.1;p.write_text(yaml.safe_dump(camera))
            with self.assertRaises(ValueError):parameters(imu,p,[-.15,0,.05,0,0,0,1],folder)
            camera['distortion_coefficients']['data'][0]=0;p.write_text(yaml.safe_dump(camera))
            with self.assertRaises(ValueError):parameters(imu,p,[-.15,0,.05,0,0,0,2],folder)


if __name__=='__main__':unittest.main()
