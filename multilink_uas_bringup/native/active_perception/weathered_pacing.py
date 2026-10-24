"""Scheduling-only tolerance for periodic view decisions; no estimator changes."""
def planner_lag_limit(paused,period):
    if not 0 < period < .25:
        raise ValueError('Two planner periods must remain below the 0.5 s freshness gate')
    return (1.75 if paused else 2.0)*period
