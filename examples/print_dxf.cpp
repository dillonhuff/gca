#include <cassert>
#include <dirent.h>
#include <iostream>

#include "dxflib/dl_dxf.h"
#include "dxflib/dl_creationadapter.h"

using namespace std;

class dxf_listener : public DL_CreationAdapter {
public:

  /**
   * Called for every code / value tuple of the DXF file. The complete DXF file
   * contents can be handled by the implemetation of this function.
   */
  virtual void processCodeValuePair(unsigned int groupCode, char* groupValue) {
    cout << "Code value pair" << endl;
  }

  /**
   * Called for every block. Note: all entities added after this
   * command go into this block until endBlock() is called.
   *
   * @see endBlock()
   */
  virtual void addBlock(const DL_BlockData& data) {
    cout << "Adding block" << endl;
    //assert(false);
  }

  /** Called to end the current block */
  virtual void endBlock() {
    cout << "End of block" << endl;
    //assert(false);
  }

  /** Called for every ellipse */
  virtual void addEllipse(const DL_EllipseData& data) {
    cout << "ERROR: Ellipses are not handled" << endl;
    assert(false);
  }

  /** Called for every spline */
  virtual void addSpline(const DL_SplineData& data) {
    cout << "ERROR: Splines are not handled" << endl;
    //assert(false);
  }
	
  /** Called for every spline control point */
  virtual void addControlPoint(const DL_ControlPointData& data) {
    cout << "ERROR: control points are not handled" << endl;
    //assert(false);
  }
	
  /** Called for every spline knot value */
  virtual void addKnot(const DL_KnotData& data) {
    cout << "ERROR: knots are not handled" << endl;
    //assert(false);
  }

  /** Called for every trace start */
  virtual void addTrace(const DL_TraceData& data) {
    assert(false);
  }
    
  /** Called for every solid start */
  virtual void addSolid(const DL_SolidData& data) {
    assert(false);
  }

  /** Called for every Multi Text entity. */
  virtual void addMText(const DL_MTextData& data) {
    cout << "ERROR: Multitext are not handled" << endl;
    //assert(false);
  }

  /**
   * Called for additional text chunks for MTEXT entities.
   * The chunks come at 250 character in size each. Note that 
   * those chunks come <b>before</b> the actual MTEXT entity.
   */
  virtual void addMTextChunk(const char* text) {
    assert(false);
  }

  /** Called for every Text entity. */
  virtual void addText(const DL_TextData& data) {
    assert(false);
  }

  /**
   * Called for every aligned dimension entity. 
   */
  virtual void addDimAlign(const DL_DimensionData& data,
			   const DL_DimAlignedData& edata) {
    assert(false);
  }
  
  /**
   * Called for every linear or rotated dimension entity. 
   */
  virtual void addDimLinear(const DL_DimensionData& data,
			    const DL_DimLinearData& edata) {
    assert(false);
  }

  /**
   * Called for every radial dimension entity. 
   */
  virtual void addDimRadial(const DL_DimensionData& data,
			    const DL_DimRadialData& edata) {
    assert(false);
  }

  /**
   * Called for every diametric dimension entity. 
   */
  virtual void addDimDiametric(const DL_DimensionData& data,
			       const DL_DimDiametricData& edata) {
    assert(false);
  }

  /**
   * Called for every angular dimension (2 lines version) entity. 
   */
  virtual void addDimAngular(const DL_DimensionData& data,
			     const DL_DimAngularData& edata) { assert(false); }

  /**
   * Called for every angular dimension (3 points version) entity. 
   */
  virtual void addDimAngular3P(const DL_DimensionData& data,
			       const DL_DimAngular3PData& edata) { assert(false); }
	
  /**
   * Called for every ordinate dimension entity. 
   */
  virtual void addDimOrdinate(const DL_DimensionData& data,
			      const DL_DimOrdinateData& edata) { assert(false); }
    
  /** 
   * Called for every leader start. 
   */
  virtual void addLeader(const DL_LeaderData& data) { assert(false); }
	
  /** 
   * Called for every leader vertex 
   */
  virtual void addLeaderVertex(const DL_LeaderVertexData& data) { assert(false); }
	
  /** 
   * Called for every hatch entity. 
   */
  virtual void addHatch(const DL_HatchData& data) { assert(false); }
	
  /** 
   * Called for every image entity. 
   */
  virtual void addImage(const DL_ImageData& data) { assert(false); }

  /**
   * Called for every image definition.
   */
  virtual void linkImage(const DL_ImageDefData& data) { assert(false); }

  /** 
   * Called for every hatch loop. 
   */
  virtual void addHatchLoop(const DL_HatchLoopData& data) { assert(false); }

  /** 
   * Called for every hatch edge entity. 
   */
  virtual void addHatchEdge(const DL_HatchEdgeData& data) { assert(false); }
	
  /** 
   * Called after an entity has been completed.  
   */
  virtual void endEntity() {
    //assert(false);
    cout << "End entity" << endl;
  }
    
  /**
   * Called for every comment in the DXF file (code 999).
   */
  virtual void addComment(const char* comment) { assert(false); }

  /**
   * Called for every vector variable in the DXF file (e.g. "$EXTMIN").
   */
  virtual void setVariableVector(const char* key, 
				 double v1, double v2, double v3, int code) {
    cout << "Set variable vector" << endl;
    //assert(false);
  }
	
  /**
   * Called for every string variable in the DXF file (e.g. "$ACADVER").
   */
  virtual void setVariableString(const char* key, const char* value, int code) {
    cout << "Setting variable string" << endl;
    //assert(false);
  }
	
  /**
   * Called for every int variable in the DXF file (e.g. "$ACADMAINTVER").
   */
  virtual void setVariableInt(const char* key, int value, int code) {
    cout << "Set variable int" << endl;
    //assert(false);
  }
	
  /**
   * Called for every double variable in the DXF file (e.g. "$DIMEXO").
   */
  virtual void setVariableDouble(const char* key, double value, int code) {
    //assert(false);
    cout << "Set variable double" << endl;
  }
	
  /**
   * Called when a SEQEND occurs (when a POLYLINE or ATTRIB is done)
   */
  virtual void endSequence() { assert(false); }

  /** Sets the current attributes for entities. */
  void setAttributes(const DL_Attributes& attrib) {
    attributes = attrib;
  }

  DL_Attributes getAttributes() {
    return attributes;
  }

  /** Sets the current attributes for entities. */
  void setExtrusion(double dx, double dy, double dz, double elevation) {
    extrusion->setDirection(dx, dy, dz);
    extrusion->setElevation(elevation);
  }

  /** @return the current attributes used for new entities. */
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
    printf("flags: %d\n", (int)data.flags);
    printAttributes();
  }

  void addVertex(const DL_VertexData& data) {
    printf("VERTEX   (%6.3f, %6.3f, %6.3f) %6.3f\n",
           data.x, data.y, data.z,
           data.bulge);
    printAttributes();
  }


  void add3dFace(const DL_3dFaceData& data) {
    cout << "ERROR: 3d faces are not supported" << endl;
    assert(false);
    // printf("3DFACE\n");
    // for (int i=0; i<4; i++) {
    //   printf("   corner %d: %6.3f %6.3f %6.3f\n", 
    // 	     i, data.x[i], data.y[i], data.z[i]);
    // }
    // printAttributes();
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

void read_dxf(char* file) {
  cout << "Reading file " << file << "...\n";
  dxf_listener* listener = new dxf_listener();
  DL_Dxf* dxf = new DL_Dxf();
  if (!dxf->in(file, listener)) {
    std::cerr << file << " could not be opened.\n";
    return;
  }
  delete dxf;
  delete listener;
  cout << "done" << endl;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: print-dxf <gcode file path>" << endl;
    return 0;
  }

  read_dxf(argv[1]);
}
