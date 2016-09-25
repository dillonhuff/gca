#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>
#include <vtkFeatureEdges.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkClipPolyData.h>
#include <vtkPlaneSource.h>
#include <vtkProperty.h>
#include <vtkCellArray.h>
#include <vtkPolyDataNormals.h>

#include <vtkAxesActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"

namespace gca {

  void highlight_cells(vtkSmartPointer<vtkPolyData> polyData,
		       const std::vector<index_t>& inds) {
    vtkSmartPointer<vtkUnsignedCharArray> colors = 
      vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");
 
    for(index_t i = 0; i < polyData->GetNumberOfCells(); i++) {
      unsigned char color[3];
      if (elem(i, inds)) {
	color[0] = 200;
	color[1] = 0;
	color[2] = 0;
      } else {
	color[0] = 0;
	color[1] = 0;
	color[2] = 200;
	
      }

      colors->InsertNextTupleValue(color);
    }
 
    polyData->GetCellData()->SetScalars(colors);
  }
  
  vtkSmartPointer<vtkActor> polydata_actor(vtkSmartPointer<vtkPolyData> polyData)
  {
    vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
 
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    return actor;
  }

  vtkSmartPointer<vtkActor> plane_actor(vtkSmartPointer<vtkPlane> pl)
  {
    double n[3];
    pl->GetNormal(n);
    double pt[3];
    pl->GetOrigin(pt);
    
    vtkSmartPointer<vtkPlaneSource> planeSource =
      vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetCenter(pt);
    planeSource->SetNormal(n);
    planeSource->Update();
 
    vtkPolyData* plane = planeSource->GetOutput();
 
    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(plane);
 
    vtkSmartPointer<vtkActor> actor =
      vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);    

    return actor;
  }
  
  void visualize_actors(const std::vector<vtkSmartPointer<vtkActor> >& actors)
  {
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

  vtkSmartPointer<vtkActor>
  highlighted_surface_actor(const std::vector<index_t>& inds,
			    const triangular_mesh& mesh) {
    auto poly_data = polydata_for_trimesh(mesh);
    highlight_cells(poly_data, inds);
    return polydata_actor(poly_data);
  }

  void vtk_debug_highlight_inds(const std::vector<index_t>& inds,
				const triangular_mesh& mesh) {
    auto surface_act = highlighted_surface_actor(inds, mesh);
    visualize_actors({surface_act});
  }

  void vtk_debug_triangles(const std::vector<triangle>& tris) {
    auto pd = polydata_for_triangles(tris);
    vtkSmartPointer<vtkActor> act = polydata_actor(pd);
    visualize_actors({act});
  }
  
  void vtk_debug_mesh(const triangular_mesh& mesh) {
    vtk_debug_highlight_inds(mesh.face_indexes(), mesh);
  }

  void vtk_debug_polygon(const oriented_polygon& p) {
    auto pd = polydata_for_polygon(p);
    debug_print_edge_summary(pd);
    auto act = polydata_actor(pd);
    visualize_actors({act});
  }

  void vtk_debug_meshes(const std::vector<const triangular_mesh*>& meshes) {
    std::vector<vtkSmartPointer<vtkActor>> actors;
    for (auto m : meshes) {
      auto p = polydata_for_trimesh(*m);
      actors.push_back(polydata_actor(p));
    }
    visualize_actors(actors);
  }

  void vtk_debug_meshes(const std::vector<triangular_mesh>& meshes) {
    std::vector<vtkSmartPointer<vtkActor>> actors;
    for (auto m : meshes) {
      auto p = polydata_for_trimesh(m);
      actors.push_back(polydata_actor(p));
    }
    visualize_actors(actors);
  }
  
  void vtk_debug_highlight_inds(const surface& surf) {
    vtk_debug_highlight_inds(surf.index_list(), surf.get_parent_mesh());
  }

  void vtk_debug_highlight_inds(const std::vector<surface>& surfs) {
    vtk_debug_highlight_inds(merge_surfaces(surfs));
  }

  bool is_closed(vtkPolyData* polydata)
  {
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

  bool has_cell_normals(vtkPolyData* polydata)
  {
    // std::cout << "Looking for cell normals..." << std::endl;
 
    // Count points
    //vtkIdType numCells = polydata->GetNumberOfCells();
    //std::cout << "There are " << numCells << " cells." << std::endl;
 
    // Count triangles
    //vtkIdType numPolys = polydata->GetNumberOfPolys();
    //std::cout << "There are " << numPolys << " polys." << std::endl;
 
    ////////////////////////////////////////////////////////////////
    // Double normals in an array
    vtkDoubleArray* normalDataDouble =
      vtkDoubleArray::SafeDownCast(polydata->GetCellData()->GetArray("Normals"));
 
    if(normalDataDouble)
      {
	//int nc = normalDataDouble->GetNumberOfTuples();
	// std::cout << "There are " << nc
	// 	  << " components in normalDataDouble" << std::endl;
	return true;
      }
 
    ////////////////////////////////////////////////////////////////
    // Double normals in an array
    vtkFloatArray* normalDataFloat =
      vtkFloatArray::SafeDownCast(polydata->GetCellData()->GetArray("Normals"));
 
    if(normalDataFloat)
      {
	//int nc = normalDataFloat->GetNumberOfTuples();
	// std::cout << "There are " << nc
	// 	  << " components in normalDataFloat" << std::endl;
	return true;
      }
 
    ////////////////////////////////////////////////////////////////
    // Point normals
    vtkDoubleArray* normalsDouble =
      vtkDoubleArray::SafeDownCast(polydata->GetCellData()->GetNormals());
 
    if(normalsDouble)
      {
	// std::cout << "There are " << normalsDouble->GetNumberOfComponents()
	// 	  << " components in normalsDouble" << std::endl;
	return true;
      }
 
    ////////////////////////////////////////////////////////////////
    // Point normals
    vtkFloatArray* normalsFloat =
      vtkFloatArray::SafeDownCast(polydata->GetCellData()->GetNormals());
 
    if(normalsFloat)
      {
	// std::cout << "There are " << normalsFloat->GetNumberOfComponents()
	// 	  << " components in normalsFloat" << std::endl;
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
 
	return true;
      }
 
    return false;
  }  
  
  void debug_print_summary(vtkPolyData* polydata) {
    cout << "# of points in polydata = " << polydata->GetNumberOfPoints() << endl;
    cout << "# of polys in polydata = " << polydata->GetNumberOfPolys() << endl;
    bool has_normals = has_cell_normals(polydata);
    cout << "Has normals ? " << (has_normals == 1 ? "True" : "False") << endl;
  }

  void debug_print_edge_summary(vtkPolyData* pdata) {
    vtkSmartPointer<vtkFeatureEdges> boundEdges = 
      vtkSmartPointer<vtkFeatureEdges>::New();
    boundEdges->FeatureEdgesOff();
    boundEdges->BoundaryEdgesOn();
    boundEdges->NonManifoldEdgesOff();
    boundEdges->SetInputData(pdata);
    boundEdges->Update();
 
    int number_of_boundary_edges =
      boundEdges->GetOutput()->GetNumberOfCells();

    vtkSmartPointer<vtkFeatureEdges> nonManifoldEdges = 
      vtkSmartPointer<vtkFeatureEdges>::New();
    nonManifoldEdges->FeatureEdgesOff();
    nonManifoldEdges->BoundaryEdgesOff();
    nonManifoldEdges->ManifoldEdgesOff();
    nonManifoldEdges->NonManifoldEdgesOn();
    nonManifoldEdges->SetInputData(pdata);
    nonManifoldEdges->Update();

    int number_of_non_manifold_edges =
      nonManifoldEdges->GetOutput()->GetNumberOfCells();

    cout << "# of boundary edges = " << number_of_boundary_edges << endl;
    cout << "# of non manifold edges = " << number_of_non_manifold_edges << endl;

    vtkIndent* indent = vtkIndent::New();
    if (number_of_non_manifold_edges > 0) {
      for (int i = 0; i < nonManifoldEdges->GetOutput()->GetNumberOfCells(); i++) {
	vtkCell* c = nonManifoldEdges->GetOutput()->GetCell(i);
	assert(c->GetCellType() == VTK_LINE);
	c->PrintSelf(cout, *indent);
      }
    }
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

  void vtk_debug_polygons(const std::vector<oriented_polygon>& polys) {
    vector<vtkSmartPointer<vtkActor>> actors;
    for (auto p : polys) {
      auto pd = polydata_for_polygon(p);
      actors.push_back(polydata_actor(pd));
    }
    visualize_actors(actors);
  }

  void vtk_debug_mesh_boundary_edges(const triangular_mesh& m) {
    auto pd = polydata_for_trimesh(m);

    vtkSmartPointer<vtkFeatureEdges> featureEdges = 
      vtkSmartPointer<vtkFeatureEdges>::New();
    featureEdges->FeatureEdgesOff();
    featureEdges->BoundaryEdgesOn();
    featureEdges->NonManifoldEdgesOff();
    featureEdges->SetInputData(pd);
    featureEdges->Update();

    auto edge_actor = polydata_actor(featureEdges->GetOutput());
    auto pd_actor = polydata_actor(pd);

    visualize_actors({pd_actor, edge_actor});
  }

  void debug_arrangement(const rigid_arrangement& a) {
    cout << "Debugging" << endl;
    vector<vtkSmartPointer<vtkActor>> actors;
    for (auto name : a.mesh_names()) {
      if (a.metadata(name).display_during_debugging) {
	auto pd = polydata_for_trimesh(a.mesh(name));
	actors.push_back(polydata_actor(pd));
      }
    }
    visualize_actors(actors);
  }

  void vtk_debug(const triangular_mesh& m,
		 const plane pl) {
    auto mpd = polydata_for_trimesh(m);
    auto mpl = vtk_plane(pl);

    visualize_actors({polydata_actor(mpd), plane_actor(mpl)});
  }

  void vtk_debug_ring(const std::vector<point>& pts) {
    auto pd = polydata_for_ring(pts);
    vector<vtkSmartPointer<vtkActor>> actors{polydata_actor(pd)};
    visualize_actors(actors);
  }
}
