# Match Ubuntu's SSE-built PCL allocation ABI while retaining upstream native code generation.
# Eigen 3.4 keeps DEFAULT_ALIGN_BYTES=32 even when MAX_ALIGN_BYTES=16 on AVX.
# glibc x86_64 malloc is 16-byte aligned; this matches installed PCL's allocator.
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MALLOC_ALREADY_ALIGNED=1)
