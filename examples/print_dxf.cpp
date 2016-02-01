#include <cassert>
#include <dirent.h>
#include <iostream>

#include "core/context.h"
#include "dxflib/dl_dxf.h"
#include "dxflib/dl_creationadapter.h"
#include "synthesis/output.h"

using namespace gca;
using namespace std;

class dxf_listener : public DL_CreationAdapter {
public:

  vector<cut*> cuts;

  int current_polyline_n;
  int polyline_vertices_left;
  point last_vertex;

  virtual void processCodeValuePair(unsigned int groupCode, char* groupValue) {}

  virtual void addBlock(const DL_BlockData& data) {
    cout << "Adding block" << endl;
  }

  virtual void endBlock() {
    cout << "End of block" << endl;
  }

  virtual void addSpline(const DL_SplineData& data) {
    cout << "Splines are not supported" << endl;
  }
	
  virtual void addControlPoint(const DL_ControlPointData& data) {
    printf("CONTROL POINT    (%6.3f, %6.3f, %6.3f)\n",
           data.x, data.y, data.z);
    printAttributes();
  }
	
  virtual void addKnot(const DL_KnotData& data) {
    printf("KNOT    %6.3f\n", data.k);
    printAttributes();
  }

  virtual void addMText(const DL_MTextData& data) {
    cout << "ERROR: Multitext are not handled" << endl;
  }

  virtual void addEllipse(const DL_EllipseData& data) { assert(false); }
  virtual void addTrace(const DL_TraceData& data) { assert(false); }
  virtual void addSolid(const DL_SolidData& data) { assert(false); }
  virtual void addMTextChunk(const char* text) { assert(false); }
  virtual void addText(const DL_TextData& data) { assert(false); }
  virtual void addDimAlign(const DL_DimensionData& data,
			   const DL_DimAlignedData& edata) { assert(false); }
  virtual void addDimLinear(const DL_DimensionData& data,
			    const DL_DimLinearData& edata) { assert(false); }
  virtual void addDimRadial(const DL_DimensionData& data,
			    const DL_DimRadialData& edata) { assert(false); }
  virtual void addDimDiametric(const DL_DimensionData& data,
			       const DL_DimDiametricData& edata) { assert(false); }
  virtual void addDimAngular(const DL_DimensionData& data,
			     const DL_DimAngularData& edata) { assert(false); }
  virtual void addDimAngular3P(const DL_DimensionData& data,
			       const DL_DimAngular3PData& edata) { assert(false); }
  virtual void addDimOrdinate(const DL_DimensionData& data,
			      const DL_DimOrdinateData& edata) { assert(false); }
  virtual void addLeader(const DL_LeaderData& data) { assert(false); }
  virtual void addLeaderVertex(const DL_LeaderVertexData& data) { assert(false); }
  virtual void addHatch(const DL_HatchData& data) { assert(false); }
  virtual void addImage(const DL_ImageData& data) { assert(false); }
  virtual void linkImage(const DL_ImageDefData& data) { assert(false); }
  virtual void addHatchLoop(const DL_HatchLoopData& data) { assert(false); }
  virtual void addHatchEdge(const DL_HatchEdgeData& data) { assert(false); }
  virtual void addComment(const char* comment) { assert(false); }


  virtual void endEntity() {
    cout << "End entity" << endl;
  }
    
  virtual void setVariableVector(const char* key, 
				 double v1, double v2, double v3, int code) {
    cout << "Set variable vector" << endl;
  }
	
  virtual void setVariableString(const char* key, const char* value, int code) {
    cout << "Setting variable string" << endl;
  }
	
  virtual void setVariableInt(const char* key, int value, int code) {
    cout << "Set variable int" << endl;
  }
	
  virtual void setVariableDouble(const char* key, double value, int code) {
    cout << "Set variable double" << endl;
  }
	
  virtual void endSequence() { assert(false); }

  void setAttributes(const DL_Attributes& attrib) {
    attributes = attrib;
  }

  DL_Attributes getAttributes() {
    return attributes;
  }

  void setExtrusion(double dx, double dy, double dz, double elevation) {
    extrusion->setDirection(dx, dy, dz);
    extrusion->setElevation(elevation);
  }

  DL_Extrusion* getExtrusion() {
    return extrusion;
  }
  
  void addLayer(const DL_LayerData& data) {
    printf("LAYER: %s flags: %d\n", data.name.c_str(), data.flags);
    printAttributes();
  }

  void addPoint(const DL_PointData& data) {
    printf("POINT    (%6.3f, %6.3f, %6.3f)\n",
           data.x, data.y, data.z);
    printAttributes();
  }

  void addLine(const DL_LineData& data) {
    printf("LINE     (%6.3f, %6.3f, %6.3f) (%6.3f, %6.3f, %6.3f)\n",
           data.x1, data.y1, data.z1, data.x2, data.y2, data.z2);
    printAttributes();
    assert(data.z1 == 0);
    assert(data.z2 == 0);
    point s(data.x1, data.y1, data.z1);
    point e(data.x2, data.y2, data.z2);
    cuts.push_back(mk_linear_cut(s, e));
  }

  void addArc(const DL_ArcData& data) {
    printf("ARC      (%6.3f, %6.3f, %6.3f) %6.3f, %6.3f, %6.3f\n",
           data.cx, data.cy, data.cz,
           data.radius, data.angle1, data.angle2);
    printAttributes();
  }

  void addCircle(const DL_CircleData& data) {
    printf("CIRCLE   (%6.3f, %6.3f, %6.3f) %6.3f\n",
           data.cx, data.cy, data.cz,
           data.radius);
    printAttributes();
  }

  void addPolyline(const DL_PolylineData& data) {
    printf("POLYLINE \n");
    printf("\t MVERTICES: %d\n", data.number);
    printf("\t MVERTICES: %d\n", data.m);
    printf("\t NVERTICES: %d\n", data.n);
    printf("flags: %d\n", (int)data.flags);
    printAttributes();
    assert(data.m == 0);
    assert(data.n == 0);
    current_polyline_n = data.n;
    polyline_vertices_left = current_polyline_n;
  }

  void addVertex(const DL_VertexData& data) {
    printf("VERTEX   (%6.3f, %6.3f, %6.3f) %6.3f\n",
           data.x, data.y, data.z,
           data.bulge);
    printAttributes();
    point v(data.x, data.y, data.z);
    if (polyline_vertices_left < current_polyline_n) {
      cuts.push_back(mk_linear_cut(last_vertex, v));
    }
    last_vertex = v;    
    polyline_vertices_left--;
  }

  void add3dFace(const DL_3dFaceData& data) {
    cout << "ERROR: 3d faces are not supported" << endl;
    assert(false);
  }

  void printAttributes() {
    printf("  Attributes: Layer: %s, ", attributes.getLayer().c_str());
    printf(" Color: ");
    if (attributes.getColor()==256)	{
      printf("BYLAYER");
    } else if (attributes.getColor()==0) {
      printf("BYBLOCK");
    } else {
      printf("%d", attributes.getColor());
    }
    printf(" Width: ");
    if (attributes.getWidth()==-1) {
      printf("BYLAYER");
    } else if (attributes.getWidth()==-2) {
      printf("BYBLOCK");
    } else if (attributes.getWidth()==-3) {
      printf("DEFAULT");
    } else {
      printf("%d", attributes.getWidth());
    }
    printf(" Type: %s\n", attributes.getLineType().c_str());
  }  
};

void collect_adjacent_cuts(const vector<cut*>& cuts,
			   vector<cut*>& cut_group,
			   unsigned i) {
  cut_group.push_back(cuts[i]);
  if (cuts.size() == i - 1) {
    return;
  }
  unsigned j = i;
  while (j < cuts.size() - 1) {
    point last_cut_end = cuts[j]->end;
    point next_cut_start = cuts[j + 1]->start;
    if (!within_eps(last_cut_end, next_cut_start)) {
      break;
    }
    cut_group.push_back(cuts[j]);
    j++;
  }
}

void append_cut_group_code(gprog* p, const vector<vector<cut*> > cut_passes) {
  point current_loc = point(0, 0, 0);
  for (unsigned i = 0; i < cut_passes.size(); i++) {
    vector<cut*> cut_group = cut_passes[i];
    from_to_with_G0(p, current_loc, cut_group.front()->start);
    for (int j = 0; j < cut_group.size(); j++) {
      point next_loc = cut_group[j]->end;
      instr* move_instr = mk_G1(next_loc.x, next_loc.y, next_loc.z, mk_omitted());
      p->push_back(move_instr);
    }
    current_loc = cut_group.back()->end;
  }
}

void make_cut_group_passes(double material_depth,
			   double cut_depth,
			   const vector<cut*>& current_group,
			   vector<vector<cut*> >& cut_group_passes) {
  assert(cut_depth < material_depth);
  double depth = material_depth - cut_depth;
  while (true) {
    vector<cut*> new_pass;
    for (unsigned i = 0; i < current_group.size(); i++) {
      cut* ct = current_group[i];
      assert(ct->is_linear_cut());
      new_pass.push_back(mk_linear_cut(point(ct->start.x, ct->start.y, depth), point(ct->end.x, ct->end.y, depth)));
    }
    cut_group_passes.push_back(new_pass);
    if (depth == 0.0) {
      break;
    }
    depth = max(0.0, depth - cut_depth);
  }
}

void append_cut_code(const dxf_listener& l, gprog* p) {
  vector<vector<cut*> > cut_groups;
  unsigned i = 0;
  while (i < l.cuts.size()) {
    vector<cut*> cut_group;
    collect_adjacent_cuts(l.cuts, cut_group, i);
    assert(cut_group.size() > 0);
    cut_groups.push_back(cut_group);
    i += cut_group.size();
  }
  double material_depth = 0.9;
  double cut_depth = 0.5;
  vector<vector<cut*> > cut_group_passes;
  for (unsigned j = 0; j < cut_groups.size(); j++) {
    vector<cut*> current_group = cut_groups[j];
    make_cut_group_passes(material_depth, cut_depth, current_group, cut_group_passes);
  }
  append_cut_group_code(p, cut_group_passes);
}

gprog* dxf_to_gcode(char* file) {
  dxf_listener* listener = new dxf_listener();
  DL_Dxf* dxf = new DL_Dxf();
  if (!dxf->in(file, listener)) {
    std::cerr << file << " could not be opened.\n";
    return NULL;
  }
  delete dxf;
  gprog* p = initial_gprog();
  append_cut_code(*listener, p);
  gprog* r = append_footer(p);
  delete listener;
  return r;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: print-dxf <gcode file path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);
  
  gprog* p = dxf_to_gcode(argv[1]);

  cout << "-- FINAL GCODE PROGRAM" << endl;
  cout << *p;
  return 0;
}
