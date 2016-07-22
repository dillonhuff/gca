#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>
#include <vtkStripper.h>
#include <vtkCutter.h>
#include <vtkFeatureEdges.h>
#include <vtkProperty.h>
#include <vtkFeatureEdges.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkClipPolyData.h>
#include <vtkPlane.h>
#include <vtkProperty.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataNormals.h>

#include <vtkAxesActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

#include "geometry/mesh_operations.h"
#include "geometry/surface.h"

namespace gca {

  bool has_cell_normals(vtkPolyData* polydata)
  {
    std::cout << "Looking for cell normals..." << std::endl;
 
    // Count points
    vtkIdType numCells = polydata->GetNumberOfCells();
    std::cout << "There are " << numCells << " cells." << std::endl;
 
    // Count triangles
    vtkIdType numPolys = polydata->GetNumberOfPolys();
    std::cout << "There are " << numPolys << " polys." << std::endl;
 
    ////////////////////////////////////////////////////////////////
    // Double normals in an array
    vtkDoubleArray* normalDataDouble =
      vtkDoubleArray::SafeDownCast(polydata->GetCellData()->GetArray("Normals"));
 
    if(normalDataDouble)
      {
	int nc = normalDataDouble->GetNumberOfTuples();
	std::cout << "There are " << nc
		  << " components in normalDataDouble" << std::endl;
	return true;
      }
 
    ////////////////////////////////////////////////////////////////
    // Double normals in an array
    vtkFloatArray* normalDataFloat =
      vtkFloatArray::SafeDownCast(polydata->GetCellData()->GetArray("Normals"));
 
    if(normalDataFloat)
      {
	int nc = normalDataFloat->GetNumberOfTuples();
	std::cout << "There are " << nc
		  << " components in normalDataFloat" << std::endl;
	return true;
      }
 
    ////////////////////////////////////////////////////////////////
    // Point normals
    vtkDoubleArray* normalsDouble =
      vtkDoubleArray::SafeDownCast(polydata->GetCellData()->GetNormals());
 
    if(normalsDouble)
      {
	std::cout << "There are " << normalsDouble->GetNumberOfComponents()
		  << " components in normalsDouble" << std::endl;
	return true;
      }
 
    ////////////////////////////////////////////////////////////////
    // Point normals
    vtkFloatArray* normalsFloat =
      vtkFloatArray::SafeDownCast(polydata->GetCellData()->GetNormals());
 
    if(normalsFloat)
      {
	std::cout << "There are " << normalsFloat->GetNumberOfComponents()
		  << " components in normalsFloat" << std::endl;
	return true;
      }
 
    /////////////////////////////////////////////////////////////////////
    // Generic type point normals
    vtkDataArray* normalsGeneric = polydata->GetCellData()->GetNormals(); //works
    if(normalsGeneric)
      {
	std::cout << "There are " << normalsGeneric->GetNumberOfTuples()
		  << " normals in normalsGeneric" << std::endl;
 
	double testDouble[3];
	normalsGeneric->GetTuple(0, testDouble);
 
	std::cout << "Double: " << testDouble[0] << " "
		  << testDouble[1] << " " << testDouble[2] << std::endl;
 
	// Can't do this:
	/*
	  float testFloat[3];
	  normalsGeneric->GetTuple(0, testFloat);
 
	  std::cout << "Float: " << testFloat[0] << " "
	  << testFloat[1] << " " << testFloat[2] << std::endl;
	*/
	return true;
      }
 
 
    // If the function has not yet quit, there were none of these types of normals
    std::cout << "Normals not found!" << std::endl;
    return false;
  }  

  vtkSmartPointer<vtkActor> polydata_actor(vtkSmartPointer<vtkPolyData> polyData) {
    vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
 
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    return actor;
  }

  void visualize_actors(const std::vector<vtkSmartPointer<vtkActor> >& actors) {
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    // Create axes
    vtkSmartPointer<vtkAxesActor> axes =
      vtkSmartPointer<vtkAxesActor>::New();

    renderer->AddActor(axes);
    for (auto actor : actors) {
      renderer->AddActor(actor);
    }
    renderer->SetBackground(.1, .2, .3);

    vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderWindow->Render();
    renderWindowInteractor->Start();
  }

  bool is_closed(vtkPolyData* polydata) {
    vtkSmartPointer<vtkFeatureEdges> featureEdges = 
      vtkSmartPointer<vtkFeatureEdges>::New();
    featureEdges->FeatureEdgesOff();
    featureEdges->BoundaryEdgesOn();
    featureEdges->NonManifoldEdgesOn();
    featureEdges->SetInputData(polydata);
    featureEdges->Update();
 
    int number_of_open_edges =
      featureEdges->GetOutput()->GetNumberOfCells();

    return number_of_open_edges == 0;
  }

  void debug_print_summary(vtkPolyData* polydata) {
    cout << "# of points in polydata = " << polydata->GetNumberOfPoints() << endl;
    cout << "# of polys in polydata = " << polydata->GetNumberOfPolys() << endl;
    bool has_normals = has_cell_normals(polydata);
    cout << "Has normals ? " << (has_normals == 1 ? "True" : "False") << endl;
  }

  void debug_print_is_closed(vtkPolyData* polydata) {
    if(is_closed(polydata)) {
      std::cout << "Surface is closed" << std::endl;
    } else {
      std::cout << "Surface is not closed" << std::endl;
    }
  }
  
  void debug_print_polydata(vtkPolyData* polydata) {
    debug_print_summary(polydata);
    debug_print_is_closed(polydata);

    vtkIndent* indent = vtkIndent::New();
    for (vtkIdType i = 0; i < polydata->GetNumberOfPolys(); i++) {
      vtkCell* c = polydata->GetCell(i);
      c->PrintSelf(cout, *indent);
      cout << "Cell type = " << c->GetCellType() << endl;
      cout << "# of points = " << c->GetNumberOfPoints() << endl;
    }
  }

  // TODO: Actually compute normals
  triangle vtkCell_to_triangle(vtkCell* c) {
    assert(c->GetCellType() == VTK_TRIANGLE);
    assert(c->GetNumberOfPoints() == 3);

    vtkTriangle* tri = dynamic_cast<vtkTriangle*>(c);
    double p0[3];
    double p1[3];
    double p2[3];
    tri->GetPoints()->GetPoint(0, p0);
    tri->GetPoints()->GetPoint(1, p1);
    tri->GetPoints()->GetPoint(2, p2);

    point v1(p0[0], p0[1], p0[2]);
    point v2(p1[0], p1[1], p1[2]);
    point v3(p2[0], p2[1], p2[2]);

    // cout << "v1 = " << v1 << endl;
    // cout << "v2 = " << v2 << endl;
    // cout << "v3 = " << v3 << endl;

    point q1 = v1 - v3;
    point q2 = v2 - v3;
    point norm = cross(q2, q1).normalize();
    //    cout << "Normal = " << norm << endl;
    return triangle(norm, v1, v2, v3);
  }

  std::vector<triangle>
  polydata_to_triangle_list(vtkPolyData* in_polydata) {
    vtkSmartPointer<vtkTriangleFilter> triangleFilter =
      vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(in_polydata);
    triangleFilter->Update();

    vtkPolyData* polydata = triangleFilter->GetOutput();

    vector<triangle> tris;
    for (vtkIdType i = 0; i < polydata->GetNumberOfPolys(); i++) {
      vtkCell* c = polydata->GetCell(i);
      tris.push_back(vtkCell_to_triangle(c));
    }
    return tris;
  }

  triangular_mesh
  trimesh_from_polydata(vtkPolyData* in_polydata) {

    auto tris = polydata_to_triangle_list(in_polydata);
    triangular_mesh m = make_mesh(tris, 0.001);
    //assert(m.is_connected());
    return m;
  }

  vtkSmartPointer<vtkPolyData>
  polydata_for_trimesh(const triangular_mesh& mesh) {
    vtkSmartPointer<vtkPoints> points =
      vtkSmartPointer<vtkPoints>::New();

    for (auto i : mesh.vertex_indexes()) {
      point p = mesh.vertex(i);
      points->InsertNextPoint(p.x, p.y, p.z);
      i++;
    }

    vtkSmartPointer<vtkCellArray> triangles =
      vtkSmartPointer<vtkCellArray>::New();

    for (auto i : mesh.face_indexes()) {
      vtkSmartPointer<vtkTriangle> triangle =
	vtkSmartPointer<vtkTriangle>::New();

      auto t = mesh.triangle_vertices(i);
      triangle->GetPointIds()->SetId(0, t.v[0]);
      triangle->GetPointIds()->SetId(1, t.v[1]);
      triangle->GetPointIds()->SetId(2, t.v[2]);

      triangles->InsertNextCell(triangle);    
    }

    // Create a polydata object
    vtkSmartPointer<vtkPolyData> polyData =
      vtkSmartPointer<vtkPolyData>::New();
 
    // Add the geometry and topology to the polydata
    polyData->SetPoints(points);
    polyData->SetPolys(triangles);

    return polyData;
  }

  void
  compute_cell_normals(vtkPolyData* pdata) {
    assert(false);
  }
  
  triangular_mesh
  clip_mesh(const triangular_mesh& m,
	    const plane pl) {

    cout << "######## Plane normal = " << pl.normal() << endl;
    cout << "######## Plane point = " << pl.pt() << endl;

    vtkSmartPointer<vtkPlane> clipPlane = 
      vtkSmartPointer<vtkPlane>::New();
    point n = pl.normal();
    point pt = pl.pt();
    clipPlane->SetNormal(n.x, n.y, n.z);
    clipPlane->SetOrigin(pt.x, pt.y, pt.z);

    vtkSmartPointer<vtkPolyData> input_mesh_data =
      polydata_for_trimesh(m);

    // Clip the source with the plane
    vtkSmartPointer<vtkClipPolyData> clipper = 
      vtkSmartPointer<vtkClipPolyData>::New();
    clipper->SetInputData(input_mesh_data);
    clipper->SetClipFunction(clipPlane);
    clipper->Update();
    
    vtkPolyData* m_data = clipper->GetOutput();

    vector<triangle> main_tris = polydata_to_triangle_list(m_data);

    // Now extract clipped edges
    vtkSmartPointer<vtkFeatureEdges> boundaryEdges =
      vtkSmartPointer<vtkFeatureEdges>::New();
    boundaryEdges->SetInputData(m_data);
    boundaryEdges->BoundaryEdgesOn();
    boundaryEdges->FeatureEdgesOff();
    boundaryEdges->NonManifoldEdgesOff();
    boundaryEdges->ManifoldEdgesOff();
 
    vtkSmartPointer<vtkStripper> boundaryStrips =
      vtkSmartPointer<vtkStripper>::New();
    boundaryStrips->SetInputConnection(boundaryEdges->GetOutputPort());
    boundaryStrips->Update();
 
    // Change the polylines into polygons
    vtkSmartPointer<vtkPolyData> boundaryPoly =
      vtkSmartPointer<vtkPolyData>::New();
    boundaryPoly->SetPoints(boundaryStrips->GetOutput()->GetPoints());
    boundaryPoly->SetPolys(boundaryStrips->GetOutput()->GetLines());

    // Triangulate the polygons
    vtkSmartPointer<vtkTriangleFilter> triangleFilter =
      vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(boundaryPoly);
    triangleFilter->Update();

    vtkPolyData* patch_data = triangleFilter->GetOutput();

    cout << "--------------- Patch triangles -------------" << endl;
    debug_print_summary(patch_data);
    debug_print_is_closed(patch_data);
    cout << "---------------------------------------------" << endl;

    vector<triangle> patch_tris = polydata_to_triangle_list(patch_data);
    concat(main_tris, patch_tris);

    cout << "# triangles in main tris = " << main_tris.size() << endl;

    triangular_mesh intermediate_mesh = make_mesh(main_tris, 0.01);

    auto pdata = polydata_for_trimesh(intermediate_mesh);

    cout << "--------------- pdata mesh -------------" << endl;
    debug_print_summary(pdata);
    debug_print_is_closed(pdata);
    cout << "---------------------------------------------" << endl;

    auto act_fp = polydata_actor(pdata);
    vector<vtkSmartPointer<vtkActor>> actors{act_fp};
    visualize_actors(actors);
    
    assert(is_closed(pdata));
    
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator =
      vtkSmartPointer<vtkPolyDataNormals>::New();

    normalGenerator->SetInputData(pdata);
    normalGenerator->SetSplitting(0);
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    pdata = normalGenerator->GetOutput();

    cout << "--------------- Final mesh -------------" << endl;
    debug_print_summary(pdata);
    debug_print_is_closed(pdata);
    cout << "---------------------------------------------" << endl;
    
    assert(is_closed(pdata));
    assert(has_cell_normals(pdata));

    triangular_mesh final_mesh = trimesh_from_polydata(pdata);
    auto rs = const_orientation_regions(final_mesh);
    assert(rs.size() == 6);
    auto sfs = outer_surfaces(final_mesh);
    cout << "# outer surfaces = " << sfs.size() << endl;
    assert(sfs.size() == 6);
    
    // auto fp = polydata_for_trimesh(final_mesh);
    // auto act_fp = polydata_actor(fp);
    // vector<vtkSmartPointer<vtkActor>> actors{act_fp};
    // visualize_actors(actors);
    
    assert(final_mesh.is_connected());
    return final_mesh;
  }

}
