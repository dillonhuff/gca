#include <vtkSTLWriter.h>
#include <vtkImplicitDataSet.h>
#include <vtkBooleanOperationPolyDataFilter.h>
#include <vtkDecimatePro.h>
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
#include <vtkProperty.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataNormals.h>

#include "geometry/mesh_operations.h"
#include "geometry/surface.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"

namespace gca {
  
  boost::optional<std::vector<point>>
  merge_center(const std::vector<point>& l, const std::vector<point>& r) {
    if (components_within_eps(l.back(), r.front(), 0.01)) {
      std::vector<point> rest(begin(r) + 1, end(r));
      std::vector<point> lc = l;
      concat(lc, rest);
      return lc;
    }
    return boost::none;
  }
  
  boost::optional<std::vector<point>>
  merge_adjacent_chains(const std::vector<point>& l, const std::vector<point>& r) {
    auto res = merge_center(l, r);
    if (res) { return *res; }
    res = merge_center(r, l);
    if (res) { return *res; }
    std::vector<point> lc = l;
    reverse(begin(lc), end(lc));
    res = merge_center(lc, r);
    if (res) { return *res; }
    std::vector<point> rc = r;
    reverse(begin(rc), end(rc));
    res = merge_center(l, rc);
    if (res) { return *res; }
    return boost::none;
  }

  bool
  try_to_merge_chains(std::vector<std::vector<point>>& plines) {
    for (unsigned i = 0; i < plines.size(); i++) {
      std::vector<point>* pi = &(plines[i]);
      for (unsigned j = 0; j < plines.size(); j++) {
	std::vector<point>* pj = &(plines[j]);
	if (i != j) {
	  boost::optional<std::vector<point>> res = merge_adjacent_chains(*pi, *pj);
	  if (res) {
	    *pi = *res;
	    plines.erase(begin(plines) + j);
	    return true;
	  }
	}
      }
    }
    return false;
  }
  
  // TODO: Templatize and merge with unordered_segments_to_index_polylines code?
  std::vector<std::vector<point>>
  connect_chains(std::vector<std::vector<point>>& chains) {
    bool merged_one = true;
    while (merged_one) {
      merged_one = try_to_merge_chains(chains);
    }
    return chains;
  }

  std::vector<oriented_polygon>
  mesh_cross_section(const triangular_mesh& m,
		     const plane p) {
    auto clipPlane = vtk_plane(p);

    vtkSmartPointer<vtkPolyData> input_mesh_data =
      polydata_for_trimesh(m);

    // Clip the source with the plane
    vtkSmartPointer<vtkClipPolyData> clipper = 
      vtkSmartPointer<vtkClipPolyData>::New();
    clipper->SetInputData(input_mesh_data);
    clipper->SetClipFunction(clipPlane);
    clipper->Update();
    
    vtkPolyData* m_data = clipper->GetOutput();

    // Now extract clipped edges
    vtkSmartPointer<vtkFeatureEdges> boundaryEdges =
      vtkSmartPointer<vtkFeatureEdges>::New();
    boundaryEdges->SetInputData(m_data);
    boundaryEdges->BoundaryEdgesOn();
    boundaryEdges->FeatureEdgesOff();
    boundaryEdges->NonManifoldEdgesOff();
    boundaryEdges->ManifoldEdgesOff();
    boundaryEdges->Update();

    vtkPolyData& edges = *(boundaryEdges->GetOutput());

    cout << "# of edges = " << edges.GetNumberOfCells() << endl;

    vector<vector<point>> ln;
    for (vtkIdType i = 0; i < edges.GetNumberOfCells(); i++) {
      vtkCell* c = edges.GetCell(i);
      line l = vtkCell_to_line(c);
      ln.push_back({l.start, l.end});
    }

    cout << "Number of initial edges = " << ln.size() << endl;

    ln = connect_chains(ln);

    cout << "Number of connected chains = " << ln.size() << endl;

    // for (auto c : ln) {
    //   cout << "CHAIN" << endl;
    //   for (auto p : c) {
    // 	cout << "---" << p << endl;
    //   }
    // }

    vector<oriented_polygon> polys;
    for (auto pt : ln) {
      auto r = pt;

      DBG_ASSERT(components_within_eps(r.front(), r.back(), 0.01));

      r.pop_back();
      
      polys.push_back(oriented_polygon(point(0, 0, 1), r));
    }

    return polys;
  }

  // TOOD: Simplify this atrocity of a function
  triangular_mesh
  clip_mesh(const triangular_mesh& m,
	    const plane pl)
  {

    auto clipPlane = vtk_plane(pl);

    vtkSmartPointer<vtkPolyData> input_mesh_data =
      polydata_for_trimesh(m);
    
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator =
      vtkSmartPointer<vtkPolyDataNormals>::New();

    normalGenerator->SetInputData(input_mesh_data);
    normalGenerator->SetSplitting(0);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    input_mesh_data = normalGenerator->GetOutput();
    
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

    vector<triangle> patch_tris = polydata_to_triangle_list(patch_data);
    concat(main_tris, patch_tris);

    triangular_mesh intermediate_mesh = make_mesh_no_winding_check(main_tris, 0.01);

    auto pdata = polydata_for_trimesh(intermediate_mesh);

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator2 =
      vtkSmartPointer<vtkPolyDataNormals>::New();

    normalGenerator2->SetInputData(pdata);
    normalGenerator2->SetSplitting(0);
    normalGenerator2->ComputePointNormalsOn();
    normalGenerator2->ComputeCellNormalsOn();
    normalGenerator2->Update();

    pdata = normalGenerator2->GetOutput();

    assert(is_closed(pdata));

    double target =
      1.0 - (static_cast<double>(pdata->GetNumberOfPolys()) - 12.0) / 12.0;
    vtkSmartPointer<vtkDecimatePro> decimate =
      vtkSmartPointer<vtkDecimatePro>::New();
    decimate->SetInputData(pdata);
    decimate->SetTargetReduction(target);
    decimate->Update();

    pdata = decimate->GetOutput();

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator3 =
      vtkSmartPointer<vtkPolyDataNormals>::New();

    normalGenerator3->SetInputData(pdata);
    normalGenerator3->SetSplitting(0);
    normalGenerator3->ComputePointNormalsOn();
    normalGenerator3->ComputeCellNormalsOn();
    normalGenerator3->Update();

    pdata = normalGenerator3->GetOutput();

    assert(is_closed(pdata));
    assert(has_cell_normals(pdata));

    triangular_mesh final_mesh = trimesh_for_polydata(pdata);

    assert(final_mesh.is_connected());

    return final_mesh;
  }

  triangular_mesh
  boolean_difference(const triangular_mesh& a, const triangular_mesh& b) {
    auto a_poly = polydata_for_trimesh(a);

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator =
      vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(a_poly);
    normalGenerator->SetSplitting(0);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    a_poly = normalGenerator->GetOutput();


    auto b_poly = polydata_for_trimesh(b);
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator2 =
      vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator2->SetInputData(b_poly);
    normalGenerator2->SetSplitting(0);
    normalGenerator2->ComputePointNormalsOn();
    normalGenerator2->ComputeCellNormalsOn();
    normalGenerator2->Update();

    b_poly = normalGenerator2->GetOutput();

    vtkSmartPointer<vtkBooleanOperationPolyDataFilter> booleanOperation =
      vtkSmartPointer<vtkBooleanOperationPolyDataFilter>::New();
    booleanOperation->SetOperationToDifference();
    booleanOperation->SetInputData( 0, a_poly );
    booleanOperation->SetInputData( 1, b_poly );
    booleanOperation->Update();

    vtkPolyData* res_poly = booleanOperation->GetOutput();
    debug_print_summary(res_poly);
    debug_print_is_closed(res_poly);
    debug_print_edge_summary(res_poly);
    auto rp = vtkSmartPointer<vtkPolyData>::New();
    rp->DeepCopy(res_poly);
    auto actor = polydata_actor(rp);
    visualize_actors({actor});
    

    triangular_mesh result = trimesh_for_polydata(res_poly);
    return result;
  }

  void write_mesh_as_stl(const triangular_mesh& m,
			 const std::string& file_name) {
    auto mesh_data = polydata_for_trimesh(m);
    
    vtkSmartPointer<vtkSTLWriter> stlWriter =
      vtkSmartPointer<vtkSTLWriter>::New();
    stlWriter->SetFileName(file_name.c_str());
    stlWriter->SetInputData(mesh_data);
    stlWriter->Write();
  }

}
