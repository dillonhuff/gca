#include "synthesis/dxf_reader.h"
#include "synthesis/shapes_to_gcode.h"

namespace gca {

  class dxf_reader : public DL_CreationAdapter {
  public:

    bool log;

    dxf_reader(bool plog) : log(plog) {}

    vector<hole_punch*> hole_punches;

    vector<cut*> cuts;

    vector<b_spline*> splines;

    int current_polyline_n;
    int polyline_vertices_left;
    point last_vertex;
    unsigned num_knots;
    unsigned num_control_points;

    virtual void addSpline(const DL_SplineData& data) {
      if (log) {
	printf("SPLINE\n");
	printf("\tDEGREE:                 %d\n", data.degree);
	printf("\tNUM KNOTS:              %d\n", data.nKnots);
	printf("\tNUM CONTROL POINTS:     %d\n", data.nControl);
	printAttributes();
      }
      assert(data.nKnots == data.nControl + data.degree + 1);
      num_knots = data.nKnots;
      num_control_points = data.nControl;
      splines.push_back(b_spline::make(data.degree));
    }

    virtual void addControlPoint(const DL_ControlPointData& data) {
      if (log) {
	printf("CONTROL POINT    (%6.3f, %6.3f, %6.3f)\n",
	       data.x, data.y, data.z);
	printAttributes();
      }
      assert(splines.back()->num_control_points() < num_control_points);
      splines.back()->push_control_point(point(data.x, data.y, data.z));
    }
	
    virtual void addKnot(const DL_KnotData& data) {
      if (log) {
	printf("KNOT    %6.4f\n", data.k);
	printAttributes();
      }
      assert(splines.back()->num_knots() < num_knots);
      splines.back()->push_knot(data.k);
    }

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
      if (log) {
	printf("LAYER: %s flags: %d\n", data.name.c_str(), data.flags);
	printAttributes();
      }
    }

    void addPoint(const DL_PointData& data) {
      if (log) {
	printf("POINT    (%6.3f, %6.3f, %6.3f)\n",
	       data.x, data.y, data.z);
	printAttributes();
      }
    }

    void addLine(const DL_LineData& data) {
      if (log) {
	printf("LINE     (%6.3f, %6.3f, %6.3f) (%6.3f, %6.3f, %6.3f)\n",
	       data.x1, data.y1, data.z1, data.x2, data.y2, data.z2);
	printAttributes();
      }
      assert(data.z1 == 0);
      assert(data.z2 == 0);
      point s(data.x1, data.y1, data.z1);
      point e(data.x2, data.y2, data.z2);
      cuts.push_back(linear_cut::make(s, e));
    }

    void addArc(const DL_ArcData& data) {
      if (log) {
	printf("ARC      (%6.3f, %6.3f, %6.3f) %6.3f, %6.3f, %6.3f\n",
	       data.cx, data.cy, data.cz,
	       data.radius, data.angle1, data.angle2);
	printAttributes();
      }
    }

    void addCircle(const DL_CircleData& data) {
      if (log) {
	printf("CIRCLE   (%6.3f, %6.3f, %6.3f) %6.3f\n",
	       data.cx, data.cy, data.cz,
	       data.radius);
	printAttributes();
      }
      assert(data.cz == 0);
      hole_punches.push_back(hole_punch::make(point(data.cx, data.cy, data.cz), data.radius));
    }

    void addPolyline(const DL_PolylineData& data) {
      if (log) {
	printf("POLYLINE \n");
	printf("\t MVERTICES: %d\n", data.number);
	printf("\t MVERTICES: %d\n", data.m);
	printf("\t NVERTICES: %d\n", data.n);
	printf("flags: %d\n", (int)data.flags);
	printAttributes();
      }
      assert(data.m == 0);
      assert(data.n == 0);
      current_polyline_n = data.n;
      polyline_vertices_left = current_polyline_n;
    }

    void addVertex(const DL_VertexData& data) {
      if (log) {
	printf("VERTEX   (%6.3f, %6.3f, %6.3f) %6.3f\n",
	       data.x, data.y, data.z,
	       data.bulge);
	printAttributes();
      }
      point v(data.x, data.y, data.z);
      if (polyline_vertices_left < current_polyline_n) {
	cuts.push_back(linear_cut::make(last_vertex, v));
      }
      last_vertex = v;    
      polyline_vertices_left--;
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
    void add3dFace(const DL_3dFaceData& data) { assert(false); }
    virtual void endSequence() { assert(false); }

    virtual void processCodeValuePair(unsigned int groupCode, char* groupValue) {}
    virtual void addBlock(const DL_BlockData& data) {}
    virtual void endBlock() {}
    virtual void endEntity() {}
    virtual void addMText(const DL_MTextData& data) {}
    virtual void setVariableVector(const char* key, 
				   double v1, double v2, double v3, int code) {}
    virtual void setVariableString(const char* key, const char* value, int code) {}
    virtual void setVariableInt(const char* key, int value, int code) {}
    virtual void setVariableDouble(const char* key, double value, int code) {}

  };

  shape_layout read_dxf(const char* file, bool log) {
    dxf_reader* listener = new (allocate<dxf_reader>()) dxf_reader(log);
    DL_Dxf* dxf = new (allocate<DL_Dxf>()) DL_Dxf();
    if (!dxf->in(file, listener)) {
      std::cerr << file << " could not be opened.\n";
      assert(false);
    }
    shape_layout shapes_to_cut(listener->cuts,
			       listener->hole_punches,
			       listener->splines);
    return shapes_to_cut;
  }

}
