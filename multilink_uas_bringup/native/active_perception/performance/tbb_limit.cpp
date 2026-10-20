// Process-local worker cap for small GTSAM graphs; opt-in through LD_PRELOAD.
#include <tbb/global_control.h>
#include <cstdlib>
#include <algorithm>
namespace {
size_t limit(){const char* p=std::getenv("UAS_TBB_THREADS");return p?std::max(1,std::atoi(p)):1;}
tbb::global_control cap(tbb::global_control::max_allowed_parallelism,limit());
}
