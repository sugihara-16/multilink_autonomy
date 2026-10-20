"""Exercise draining of real child processes before their service is removed."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from simulation_shutdown import stop_world_clients


class WorldClientShutdownTest(unittest.TestCase):
    def test_final_request_finishes_before_service_shutdown(self):
        with tempfile.TemporaryDirectory() as folder:
            service = Path(folder) / 'service_alive'
            service.touch()
            code = '''import signal,time,sys
from pathlib import Path
service=Path(sys.argv[1]);stopping=False
def stop(*unused):
 global stopping
 stopping=True
signal.signal(signal.SIGINT,stop)
print('ready',flush=True)
while not stopping:time.sleep(.005)
time.sleep(.05)
if not service.exists():sys.exit(7)
service.with_name('final_request').write_text('completed')
'''
            proc = subprocess.Popen([sys.executable, '-c', code, str(service)], stdout=subprocess.PIPE, text=True, start_new_session=True)
            try:
                self.assertEqual(proc.stdout.readline().strip(), 'ready')
                result = stop_world_clients([('pacing', proc)])
                service.unlink()
                self.assertTrue(service.with_name('final_request').exists())
                self.assertEqual(result, [dict(name='pacing', exit_code=0, timed_out=False)])
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                proc.stdout.close()

    def test_existing_failure_is_preserved(self):
        proc = subprocess.Popen([sys.executable, '-c', 'raise SystemExit(3)'], start_new_session=True)
        proc.wait()
        self.assertEqual(stop_world_clients([('pacing', proc)])[0]['exit_code'], 3)

    def test_timeout_is_not_reported_as_success(self):
        proc = subprocess.Popen([sys.executable, '-c', "import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);print('ready',flush=True);time.sleep(5)"], start_new_session=True, stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(proc.stdout.readline().strip(), 'ready')
            report = stop_world_clients([('pacing', proc)], timeout=.05)[0]
            self.assertTrue(report['timed_out'])
            self.assertIsNone(report['exit_code'])
        finally:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
            proc.stdout.close()


if __name__ == '__main__':
    unittest.main()
