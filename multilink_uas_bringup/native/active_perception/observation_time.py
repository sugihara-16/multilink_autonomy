"""Time association for asynchronously delivered measured observations."""
def latest_not_after(records, time_s, maximum_age_s=.5):
    candidates=[x for x in records if -2e-9 <= time_s-x[0]*1e-9 < maximum_age_s]
    return max(candidates,key=lambda x:x[0]) if candidates else None
