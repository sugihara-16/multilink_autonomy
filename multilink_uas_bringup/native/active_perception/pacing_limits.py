"""Wall-time pacing thresholds must permit a healthy periodic worker to resume."""

def planner_lag_limit(paused,period):
    if not 0<period<1/3:raise ValueError('Planner period must keep its stop lag below 0.5 s')
    # Never require a new command before it is scheduled: a frozen clock
    # cannot advance a healthy age of, e.g., 0.136 s to the next 0.2 s tick.
    return (1.25 if paused else 1.5)*period
