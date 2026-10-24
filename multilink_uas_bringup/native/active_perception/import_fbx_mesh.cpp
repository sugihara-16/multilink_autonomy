// Convert a transformed, triangulated FBX scene to OBJ without changing UVs.
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <fstream>
#include <iostream>
#include <iomanip>
int main(int argc, char **argv) {
  if (argc != 3) { std::cerr << "usage: import_fbx_mesh input.fbx output.obj\n"; return 2; }
  const aiScene *s = aiImportFile(argv[1], aiProcess_Triangulate | aiProcess_PreTransformVertices);
  if (!s) { std::cerr << aiGetErrorString() << '\n'; return 1; }
  std::ofstream o(argv[2]); o << std::setprecision(10);
  size_t offset = 1;
  for (unsigned k = 0; k < s->mNumMeshes; ++k) {
    const aiMesh *m = s->mMeshes[k];
    if (!m->HasTextureCoords(0)) { std::cerr << "Missing UVs\n"; aiReleaseImport(s); return 1; }
    o << "o mesh_" << k << '\n';
    for (unsigned i = 0; i < m->mNumVertices; ++i) {
      const auto &v = m->mVertices[i]; o << "v " << v.x << ' ' << v.y << ' ' << v.z << '\n';
    }
    for (unsigned i = 0; i < m->mNumVertices; ++i) {
      const auto &t = m->mTextureCoords[0][i]; o << "vt " << t.x << ' ' << t.y << '\n';
    }
    for (unsigned i = 0; i < m->mNumFaces; ++i) {
      const auto &f = m->mFaces[i];
      if (f.mNumIndices != 3) continue;
      o << 'f';
      for (unsigned j = 0; j < 3; ++j) { auto n = offset + f.mIndices[j]; o << ' ' << n << '/' << n; }
      o << '\n';
    }
    offset += m->mNumVertices;
  }
  bool ok = bool(o); aiReleaseImport(s); return ok ? 0 : 1;
}
