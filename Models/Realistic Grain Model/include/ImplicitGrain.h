#ifndef IMPLICIT_GRAIN_H
#define IMPLICIT_GRAIN_H

#include <vtkImplicitFunction.h>
#include <vtkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkPolyData.h>

class ImplicitGrain : public vtkImplicitFunction
{
public:
  static ImplicitGrain* New();
  vtkTypeMacro(ImplicitGrain, vtkImplicitFunction);

  // Overrides the core distance function:
  double EvaluateFunction(double xyz[3]) override;

  // Overrides EvaluateGradient for normals or gradients:
  void EvaluateGradient(double xyz[3], double g[3]) override {
    const double h = 1e-4;
    double f0 = this->EvaluateFunction(xyz);

    double xh[3] = {xyz[0] + h, xyz[1], xyz[2]};
    double yh[3] = {xyz[0], xyz[1] + h, xyz[2]};
    double zh[3] = {xyz[0], xyz[1], xyz[2] + h};

    g[0] = (this->EvaluateFunction(xh) - f0) / h;
    g[1] = (this->EvaluateFunction(yh) - f0) / h;
    g[2] = (this->EvaluateFunction(zh) - f0) / h;
  }

  void UpdateTransform();

  // Setters
  void SetImplicit2D(vtkImplicitPolyDataDistance* implicit) { 
    this->Implicit2D = implicit; 
  }
  void SetA(double a) { 
    this->A = a; 
  }
  void SetB(double b) {
    this->B = b; 
  }
  void SetC(double c) { 
    this->C = c; 
  }
  void SetCenter(double x, double y, double z) {
    this->center[0] = x;
    this->center[1] = y;
    this->center[2] = z;
    UpdateTransform();
  }
  void SetRotation(double x, double y, double z) {
    this->rotation[0] = x;
    this->rotation[1] = y;
    this->rotation[2] = z;
    UpdateTransform();
  }

  // Getters
  double * GetCenter();
  double GetRadius(); // Culling Sphere Radius

protected:
  ImplicitGrain();
  ~ImplicitGrain() override;

private:
  ImplicitGrain(const ImplicitGrain&) = delete;
  void operator=(const ImplicitGrain&) = delete;

  double A, B, C;
  double center[3] = {0.0, 0.0, 0.0};
  double rotation[3] = {0.0, 0.0, 0.0};
  vtkSmartPointer<vtkTransform> InverseRotationTransform;
  vtkSmartPointer<vtkImplicitPolyDataDistance> Implicit2D;
};

#endif