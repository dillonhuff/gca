#ifndef GCA_RIGID_ARRANGEMENT_H
#define GCA_RIGID_ARRANGEMENT_H

#include <map>
#include <memory>
#include <vector>

#include "geometry/homogeneous_transformation.h"
#include "geometry/surface.h"
#include "utils/check.h"

namespace gca {

  class rigid_arrangement {
    std::vector<std::unique_ptr<triangular_mesh>> meshes;
    std::map<std::string, triangular_mesh*> name_index;
    
  public:
    void insert(const std::string& name,
		const triangular_mesh& m) {
      meshes.push_back(unique_ptr<triangular_mesh>(new triangular_mesh(m)));
      std::unique_ptr<triangular_mesh>* res = &(meshes.back());
      name_index[name] = res->get();
    }

    void insert(const std::string& name,
		const homogeneous_transform& t,
		const triangular_mesh& m) {
      insert(name, apply(t, m));
    }

    const std::vector<std::string> mesh_names() const {
      std::vector<std::string> names;
      for (auto p : name_index) {
	names.push_back(p.first);
      }
      return names;
    }

    const triangular_mesh& mesh(const std::string name) const {
      auto r = name_index.find(name);
      DBG_ASSERT(r != end(name_index));
      return *(r->second);
    }

  };
}

#endif
