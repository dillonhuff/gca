#include <vtkVersion.h>
#include <vtkBooleanOperationPolyDataFilter.h>
 
#include <vtkActor.h>
#include <vtkMassProperties.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
 
int main(int argc, char *argv[]) {
  vtkSmartPointer<vtkPolyData> input1;
  vtkSmartPointer<vtkPolyData> input2;
 
  std::string operation("intersection");
  vtkSmartPointer<vtkSphereSource> sphereSource1 =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource1->SetCenter(.25, 0, 0);
  sphereSource1->Update();
  input1 = sphereSource1->GetOutput();
 
  vtkSmartPointer<vtkSphereSource> sphereSource2 =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource2->Update();
  input2 = sphereSource2->GetOutput();

  vtkSmartPointer<vtkMassProperties> mass =
    vtkMassProperties::New();
  mass->SetInputData(input1);
  mass->Update();

  double volume = mass->GetVolume();

  std::cout << "Volume of input 1 = " << volume << std::endl;

  
  return EXIT_SUCCESS;
}
