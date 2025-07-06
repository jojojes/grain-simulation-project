#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ImplicitGrain.h"

#include <QMainWindow>

#include <vector>
#include <array>
#include <memory>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <QVTKInteractor.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkImplicitFunction.h>
#include <vtkActor.h>
#include <vtkTransform.h>
#include <vtkSphere.h>
#include <vtkCubeSource.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
	Ui::MainWindow *ui;

	// FULL MODEL
	double grainLength = 0.7;
	std::vector<vtkSmartPointer<ImplicitGrain>> grains;

	//VTK
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> mRenderWindow;
	vtkSmartPointer<vtkRenderer> mRenderer;
	vtkSmartPointer<QVTKInteractor> mInteractor;
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> mInteractorStyle;

	//Simulator Parameters
	double lowerBoundx = -10;
	double upperBoundx = 10;
	double lowerBoundy = -10;
	double upperBoundy = 10;
	double lowerBoundz = -10;
	double upperBoundz = 10;
	int moveTolerance = 10; // The amount of sequential moves before decreasing size of objects
	int decreaseTolerance = 5; // The amount of size decreases before decreasing overlap tolerance and converging growth/shrink rates to 1
	int sequenceMax = 10; // The amount of iterations the simulation spends decreasing tolerances and converging on a minimal porosity
	
	// User Parameters
	int grainCount = 1;
  	double grainSize = 8;
	double grainDistribution = 0.7;
	double targetPorosity = 0;
	

	//Simulator Variables and Objects
	double startOverlapTolerance = 0.3; // The fraction of allowed penetration depth before objects need to be moved
	double startGrowthFactor = 0.5; // The growth rate
	double startShrinkFactor = 0.25; // The shrink rate
	int moveCount = 0;
	int decreaseCount = 0;
	int sequenceCount = 0;
	double lowerBound = -5;
	double upperBound = 5;
	int boxesSize;
	double currentPorosity = 1;
	double minPorosity = 1;
	double overlapTolerance = 0.3;
	double growthFactor = 0.5;
	double shrinkFactor = 0.25;
	std::vector<std::array<double,3>> centers;
	std::vector<std::array<double,3>> rotations;
	std::vector<vtkSmartPointer<vtkTransform>> transforms;
	std::vector<vtkSmartPointer<vtkSphere>> spheres;
	std::vector<vtkSmartPointer<vtkActor>> actors;
	std::vector<std::array<double,6>> directions;
	std::vector<std::vector<std::vector<std::array<int,10>>>> boxes;
	vtkSmartPointer<vtkCubeSource> boundingCube;
	vtkSmartPointer<vtkActor> boundingCubeActor;

	

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
	void onSimulateStepClick();
	void onSimulateStartClick();
	void onSimulateClick();
	void simulationStep();
	void renderState();
	void generateInitial();
	vtkSmartPointer<vtkImplicitPolyDataDistance> getContourImplicit();
	void increaseSize();
	void decreaseSize();
	std::vector<std::array<double,3>> generateRandomPoints(int N);
	std::vector<std::array<double,3>> generateRandomRotation(int N);
  	double checkCollision(int id1, int id2);
	double boundSphereX(double coord, double radius);
	double boundSphereY(double coord, double radius);
	double boundSphereZ(double coord, double radius);
	bool findOverlaps();
	void moveGrains();
	void attemptMove(int i, double maxOverlap, double totalOverlap, double collisions, double dx, double dy, double dz);
	double getPorosity();
};

#endif // MAINWINDOW_H
