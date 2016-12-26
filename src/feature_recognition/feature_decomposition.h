#ifndef GCA_FEATURE_DECOMPOSITION_H
#define GCA_FEATURE_DECOMPOSITION_H

#include <deque>

#include "feature_recognition/prismatic_feature.h"
#include "synthesis/contour_planning.h"

namespace gca {

  class feature_decomposition {
  protected:
    feature* feat;

    std::vector<feature_decomposition*> children;

  public:
    feature_decomposition() :
      feat(nullptr) {}

    feature_decomposition(feature* p_feat) :
      feat(p_feat) {}

    const gca::feature* feature() const { return feat; }
    gca::feature* feature() { return feat; }
    
    void add_child(feature_decomposition* decomp)
    { children.push_back(decomp); }

    void delete_child(const unsigned i)
    { children.erase(begin(children) + i); }

    void set_feature(class feature* f) {
      feat = f;
    }

    unsigned num_children() const { return children.size(); }

    feature_decomposition* child(unsigned i) const {
      DBG_ASSERT(i < children.size());
      return children[i];
    }

    unsigned num_features() const {
      unsigned num_child_features = 0;
      for (auto c : children) {
	num_child_features += c->num_features();
      }
      return (feat == nullptr ? 0 : 1) + num_child_features;
    }

    unsigned num_levels() const {
      if (children.size() > 0) {
	unsigned max_child_num_levels =
	  max_e(children,
		[](const feature_decomposition* d) { return d->num_levels(); })->num_levels();
	return 1 + max_child_num_levels;
      }

      return 1;
    }
  };

  gca::feature* node_value(feature_decomposition* f);

  template<typename T, typename F>
  void delete_leaves(T* tree, F should_delete) {
    bool deleted_one = true;

    while (deleted_one) {
      deleted_one = false;

      for (unsigned i = 0; i < tree->num_children(); i++) {
	T* next = tree->child(i);
	if (next->num_children() == 0 && should_delete(node_value(next))) {
	  tree->delete_child(i);
	  deleted_one = true;
	  break;
	}
      }

    }

    for (unsigned i = 0; i < tree->num_children(); i++) {
      delete_leaves(tree->child(i), should_delete);
    }
  }

  template<typename T, typename F>
  void delete_internal_nodes(T* tree, F should_delete) {
    bool deleted_one = true;

    while (deleted_one) {
      deleted_one = false;
      
      for (unsigned i = 0; i < tree->num_children(); i++) {
	T* next = tree->child(i);
	if (next->num_children() != 0 && should_delete(node_value(next))) {
	  tree->delete_child(i);
	  deleted_one = true;
	  break;
	}
      }

    }

    for (unsigned i = 0; i < tree->num_children(); i++) {
      delete_internal_nodes(tree->child(i), should_delete);
    }
  }
  
  template<typename T, typename F>
  void delete_nodes(T* tree, F should_delete) {
    bool deleted_one = true;

    while (deleted_one) {
      deleted_one = false;
      
      for (unsigned i = 0; i < tree->num_children(); i++) {
	T* next = tree->child(i);
	if (should_delete(node_value(next))) {
	  tree->delete_child(i);
	  deleted_one = true;
	  break;
	}
      }

    }

    for (unsigned i = 0; i < tree->num_children(); i++) {
      delete_nodes(tree->child(i), should_delete);
    }
  }
  
  template<typename T, typename F>
  void traverse_bf(T* tree, F f) {
    std::deque<T*> active{tree};
    while (active.size() > 0) {
      T* next = active.front();
      f(node_value(next));

      active.pop_front();

      for (auto i = 0; i < next->num_children(); i++) {
	active.push_back(next->child(i));
      }
    }
  }

  template<typename T, typename F>
  void traverse_leaves_bf(T* tree, F f) {
    std::deque<T*> active{tree};
    while (active.size() > 0) {
      T* next = active.front();

      if (next->num_children() == 0) {
	f(node_value(next));
      }

      active.pop_front();

      for (auto i = 0; i < next->num_children(); i++) {
	active.push_back(next->child(i));
      }
    }
  }

  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& m, const point n);

  vector<feature*> collect_features(feature_decomposition* f);

  surface_levels
  initial_surface_levels(const std::vector<triangular_mesh>& m,
			 const point n);

  point normal(feature_decomposition* f);

  std::vector<feature*> collect_leaf_features(feature_decomposition* f);

  feature* parent_feature(feature* f, feature_decomposition* decomp);

  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& stock,
			      const std::vector<triangular_mesh>& meshes,
			      const point n);
  
  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& stock,
			      const triangular_mesh& m,
			      const point n);

  oriented_polygon to_oriented_polygon(const labeled_polygon_3& p);

}

#endif
