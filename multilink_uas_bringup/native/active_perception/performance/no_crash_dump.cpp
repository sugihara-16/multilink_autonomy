// Experiment-only: keep failure exit codes and stderr, but prevent WSL from
// writing multi-gigabyte process images to the nearly-full Windows drive.
// This is process-local and does not change the OS core_pattern or limits.
#include <sys/prctl.h>
#include <unistd.h>
#include <cstdlib>

__attribute__((constructor)) static void prevent_experiment_core_dump() {
  if (prctl(PR_SET_DUMPABLE, 0) != 0 || prctl(PR_GET_DUMPABLE) != 0) {
    const char error[] = "Experiment core-dump guard failed\n";
    write(STDERR_FILENO, error, sizeof(error)-1);
    _exit(126);
  }
}
