#include "system/write_ply.h"

#include <fstream>

namespace gca {

  void write_to_ply(const triangular_mesh& m, const std::string& path) {
    ofstream out_stream(path + ".ply", std::ofstream::out);
    out_stream << "ply" << endl;
    out_stream << "format ascii 1.0" << endl;
    out_stream << "element vertex " << m.vertex_list().size() << endl;
    out_stream << "property float32 x" << endl;
    out_stream << "property float32 y" << endl;
    out_stream << "property float32 z" << endl;
    out_stream << "element face " << m.triangle_verts().size() << endl;
    out_stream << "property list uint8 int32 vertex_index" << endl;
    out_stream << "end_header" << endl;

    // Vertex list
    for (auto v : m.vertex_list()) {
      out_stream << v.x << " " << v.y << " " << v.z << endl;
    }

    for (auto t : m.triangle_verts()) {
      out_stream << 3 << " " << t.v[0] << " " << t.v[1] << " " << t.v[2] << endl;
    }
    // for (int i = 0; i < m.vertex_list().size(); i++) {
    //   const double* p = mesh->GetVertex(i);
    //   out_stream << p[0] << " " << p[1] << " " << p[2];
    // }

    // Face list
    // for (int i = 0; i < mesh->GetNFaces(); i++) {
    //   const Element* e = mesh->GetFace(i);
    //   assert(e->GetNVertices() == 3);
    //   const int* verts = e->GetVertices();
    //   out_stream << 3 << " " << verts[0] << " " << verts[1] << " " << verts[2] << endl;
    // }
    
  }

}
