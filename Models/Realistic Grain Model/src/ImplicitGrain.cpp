#include "ImplicitGrain.h"
#include <vtkObjectFactory.h>
#include <cmath>

vtkStandardNewMacro(ImplicitGrain);

ImplicitGrain::ImplicitGrain()
  : A(1.0), B(1.0), C(1.0), Implicit2D(nullptr) {
  InverseRotationTransform = vtkSmartPointer<vtkTransform>::New();
}

ImplicitGrain::~ImplicitGrain() = default;

double * ImplicitGrain::GetCenter() {
  return center;
}

double ImplicitGrain::GetRadius() {
  return std::max(B, C);
}

// Updates the transformation for rotation
void ImplicitGrain::UpdateTransform() {
  vtkSmartPointer<vtkTransform> Transform = vtkSmartPointer<vtkTransform>::New();
  Transform->PostMultiply();
  Transform->Translate(-center[0], -center[1], -center[2]);
  Transform->RotateX(rotation[0]);
  Transform->RotateY(rotation[1]);
  Transform->RotateZ(rotation[2]);
  Transform->Inverse();
  InverseRotationTransform->DeepCopy(Transform);
}

// Evaluates a point to be inside (negative) or outside (positive)
double ImplicitGrain::EvaluateFunction(double xyz[3]) {

  double local[3];
  this->InverseRotationTransform->TransformPoint(xyz, local);

  double x = local[0];
  double y = local[1];
  double z = local[2];

  // Computes cross-section radii at z:
  double frac = 1.0 - (z*z)/(C*C);
  if (frac < 0.0) { 
      frac = 0.0; 
  }
  double scale = std::sqrt(frac);
  double Az = A * scale; // semi-axis in X at this z
  double Bz = B * scale; // semi-axis in Y at this z

  if (Az <= 0.01) {
    Az = 0.01;
  }

  if (Bz <= 0.01) {
    Bz = 0.01;
  }

  double u = x / Az;
  double v = y / Bz;

  // 2D signed-distance:
  double pt2D[3] = { u, v, 0.0 };
  double d2d = this->Implicit2D->FunctionValue(pt2D);
  double dz = 1.0 - (std::abs(z) / C);
  return std::min(d2d, dz);
}