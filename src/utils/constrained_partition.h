#ifndef GCA_CONSTRAINED_PARTITION_H
#define GCA_CONSTRAINED_PARTITION_H

#include "utils/partition.h"

namespace gca {

  template<typename Item, typename Bucket>
  class constrained_partition {
  protected:
    relation<Item, Bucket> constraint;
    partition<Item, Bucket> part;

  public:

    constrained_partition(const relation<Item, Bucket>& rel)
      : constraint(rel), part(rel.left_elems(), rel.right_elems()) {}
    
    // inline bool is_finished() const { return false; }

    Bucket bucket(const unsigned i) const { return part.bucket(i); }
    Item item(const unsigned i) const { return part.item(i); }

    std::vector<unsigned> bucket_inds() const { return part.bucket_inds(); }
    std::vector<unsigned> item_inds() const { return part.item_inds(); }

    std::vector<unsigned> items_in_bucket_inds(const unsigned i) const {
      return part.items_in_bucket_inds(i);
    }

    bool bucket_can_contain_all(const unsigned bucket_ind,
				const std::vector<unsigned>& item_inds) const {
      bool can_contain_all =
      	intersection(constraint.lefts_connected_to(bucket_ind),
      		     item_inds).size() == item_inds.size();
      return can_contain_all;
    }

    inline
    const std::vector<Item>& items() const { return part.items(); }

    std::vector<unsigned> non_empty_bucket_inds() const {
      std::vector<unsigned> all_inds = bucket_inds();
      delete_if(all_inds,
    		[this](const unsigned i)
    		{ return this->items_in_bucket_inds(i).size() == 0; });
      return all_inds;
    }
    
    void assign_item_to_bucket(const unsigned i_ind,
    			       const unsigned b_ind) {
      //rel.insert(i_ind, b_ind);
      part.assign_item_to_bucket(i_ind, b_ind);
    }

  };

}

#endif
