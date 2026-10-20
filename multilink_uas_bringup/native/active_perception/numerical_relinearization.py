"""Diagnostic: remove incremental relinearization/back-substitution cutoffs.

Keeps the objective, measurement covariance and number of optimizer iterations.
This is not a claim that three iterations reach a global or converged optimum.
"""
def apply(config):
    smoother = config['graph']['manager']['smoother']
    names = ['rotation','translation','velocity','bias_acc','bias_gyro','gravity']
    required = ['relinearize_threshold_'+name for name in names]
    if any(key not in smoother for key in required):
        raise ValueError('Expected all configured relinearization thresholds')
    for key in required:
        smoother[key] = 0.
    smoother.update(enable_partial_relinearization_check=False,
                    enable_relinearization=True,relinearize_skip=1,wildfire_threshold=0.)
