"""Emit completion after an actual planning attempt, including a no-command hold."""
def completed_attempt(function, notify):
    def wrapped(self,event):
        result=function(self,event)
        notify(self)
        return result
    return wrapped
