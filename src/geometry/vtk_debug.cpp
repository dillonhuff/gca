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

#include "geometry/vtk_debug.h"

namespace gca {

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

  
}
