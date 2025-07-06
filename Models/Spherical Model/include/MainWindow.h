#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkCamera.h>
#include <vtkPolyData.h>

#include <QFileDialog>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
	Ui::MainWindow *ui;

	//VTK
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> mRenderWindow;
	vtkSmartPointer<vtkRenderer> mRenderer;
	vtkSmartPointer<QVTKInteractor> mInteractor;
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> mInteractorStyle;

	//Simulator Program Parameters
	double lowerBoundx = -10;
	double upperBoundx = 10;
	double lowerBoundy = -10;
	double upperBoundy = 10;
	double lowerBoundz = -10;
	double upperBoundz = 10;
	int moveTolerance = 10; // The amount of sequential moves before decreasing size of objects
	int decreaseTolerance = 5; // The amount of size decreases before decreasing overlap tolerance and converging growth/shrink rates to 1
	int sequenceMax = 25; // The amount of iterations the simulation spends decreasing tolerances and converging on a minimal porosity
	
	// User Parameters
	int grainAmount = 100; // The amount of grains
  	double grainSize = 1; // The starting size of the largest grain
	double grainDistribution = 0.7; // The fraction between sizes of large and small grains
	double targetPorosity = 0; // The target final porosity to aim for
	

	//Simulator Variables and Objects
	double startOverlapTolerance = 0.2; // The fraction of allowed penetration depth before objects need to be moved
	double startGrowthFactor = 0.25; // The growth rate
	double startShrinkFactor = 0.125; // The shrink rate
	int moveCount = 0;
	int decreaseCount = 0;
	int sequenceCount = 0;
	int boxesSize;
	double currentPorosity = 1;
	double samplePorosity = 1;
	double minPorosity = 1;
	double overlapTolerance = 0.3;
	double growthFactor = 0.25;
	double shrinkFactor = 0.125;
	bool cross = false;
	std::vector<std::array<double,3>> centers;
	std::vector<vtkSmartPointer<vtkSphere>> spheres;
	std::vector<vtkSmartPointer<vtkActor>> actors;
	std::vector<std::array<double,6>> directions;
	std::vector<std::vector<std::vector<std::array<int,25>>>> boxes;
	vtkSmartPointer<vtkCubeSource> boundingCube;
	vtkSmartPointer<vtkActor> boundingCubeActor;
	vtkSmartPointer<vtkPolyData> mPolyData;
	vtkSmartPointer<vtkCutter> cutter;

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
	void onSimulateStepClick();
	void onSimulateStartClick();
	void onSimulateClick();
	void renderState();
	void simulationStep();
	void generateInitial();
	void increaseSize();
	void decreaseSize();
	std::vector<std::array<double,3>> generateRandomPoints(int N);
  	double checkCollision(int id1, int id2);
	double boundSphereX(double coord, double radius);
	double boundSphereY(double coord, double radius);
	double boundSphereZ(double coord, double radius);
	void findOverlapsNeighbors(int sphere, int bx, int by, int bz);
	bool findOverlaps();
	void moveSpheres();
	void attemptMove(int i, double maxOverlap, double totalOverlap, double collisions, double dx, double dy, double dz);
	double getTruePorosity();
	double getSamplePorosity();
	void saveScreenshot(const QString& filename);
};

#endif // MAINWINDOW_H
