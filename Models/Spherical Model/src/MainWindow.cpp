#include <MainWindow.h>

#include "ui_MainWindow.h"

#include <vtkActor.h>
#include <vtkContourFilter.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSampleFunction.h>
#include <vtkSphere.h>
#include <vtkCubeSource.h>
#include <vtkImplicitFunction.h>
#include <vtkImplicitBoolean.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkMassProperties.h>
#include <vtkMath.h>

#include <vtkAppendPolyData.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkCamera.h>
#include <vtkPolyData.h>

#include <QElapsedTimer>
#include <random>
#include <cmath>
#include <algorithm>
#include <array>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
	mRenderWindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()),
	mRenderer(vtkSmartPointer<vtkRenderer>::New()),
	mInteractor(vtkSmartPointer<QVTKInteractor>::New()),
	mInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New())
{
    ui->setupUi(this);

	// Setup the spin boxes
    ui->GrainCount_spinBox->setRange(1, 10000);
    ui->GrainCount_spinBox->setValue(grainAmount);

	ui->GrainSize_doubleSpinBox->setRange(0.01, 100.0);
    ui->GrainSize_doubleSpinBox->setSingleStep(0.01);
    ui->GrainSize_doubleSpinBox->setValue(grainSize);

	ui->GrainDistribution_doubleSpinBox->setRange(0.01, 1.0);
    ui->GrainDistribution_doubleSpinBox->setSingleStep(0.01);
    ui->GrainDistribution_doubleSpinBox->setValue(grainDistribution);

	ui->TargetPorosity_doubleSpinBox->setRange(0.01, 1.0);
    ui->TargetPorosity_doubleSpinBox->setSingleStep(0.01);
    ui->TargetPorosity_doubleSpinBox->setValue(targetPorosity);
	

	// Set up the rendering
	mRenderWindow->SetSize(750, 750);
	mRenderWindow->AddRenderer(mRenderer);
	mRenderWindow->SetInteractor(mInteractor);
	ui->openGLWidget->setRenderWindow(mRenderWindow);
	mInteractor->SetInteractorStyle(mInteractorStyle);
	mInteractor->Initialize();

	// Set the background color 
	mRenderer->SetBackground(1, 1, 1);

	// Set the UI connections
	QObject::connect(ui->SimulateStart_button, &QPushButton::clicked, this, &MainWindow::onSimulateStartClick);
	QObject::connect(ui->SimulateStep_button, &QPushButton::clicked, this, &MainWindow::onSimulateStepClick);
	QObject::connect(ui->Simulate_button, &QPushButton::clicked, this, &MainWindow::onSimulateClick);
	QObject::connect(ui->SaveImage_button, &QPushButton::clicked, [=](){
    	QString fn = QFileDialog::getSaveFileName(this, "Save Screenshot", "", "*.png");
    	if (!fn.isEmpty()) this->saveScreenshot(fn);
	});

	//generateInitial();
	//renderState();
}

MainWindow::~MainWindow() {
    delete ui;
}

// Generates a random point within the bounds
std::vector<std::array<double,3>> MainWindow::generateRandomPoints(int n)
{
    std::random_device random;
    std::mt19937 generator(random());
    std::uniform_real_distribution<double> distributionx(lowerBoundx, upperBoundx);
	std::uniform_real_distribution<double> distributiony(lowerBoundy, upperBoundy);
    std::uniform_real_distribution<double> distributionz(lowerBoundz, upperBoundz);
    std::vector<std::array<double,3>> coords;
    coords.reserve(n);
    for (int i = 0; i < n; ++i) {
        coords.push_back({distributionx(generator), distributiony(generator), distributionz(generator)});
    }
    return coords;
}

void MainWindow::generateInitial() {
	mRenderer->RemoveAllViewProps();
  	auto makeActor = [&](vtkImplicitFunction* implicit, double r, const double color[3]) {
    	// Sample the implicit function on a 3D grid
    	auto sampler = vtkSmartPointer<vtkSampleFunction>::New();
    	sampler->SetSampleDimensions(50,50,50);
    	sampler->SetImplicitFunction(implicit);
    	sampler->SetModelBounds(lowerBoundx, upperBoundx, lowerBoundy, upperBoundy, lowerBoundz, upperBoundz);

    	auto contour = vtkSmartPointer<vtkContourFilter>::New();
    	contour->SetInputConnection(sampler->GetOutputPort());
    	contour->SetValue(0, 0.0);

    	auto xfFilter = vtkSmartPointer<vtkTransformFilter>::New();
    	xfFilter->SetInputConnection(contour->GetOutputPort());
    	xfFilter->SetTransform(vtkSmartPointer<vtkTransform>::New());

    	auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    	mapper->SetInputConnection(xfFilter->GetOutputPort());
    	mapper->ScalarVisibilityOff();

    	auto actor = vtkSmartPointer<vtkActor>::New();
    	actor->SetMapper(mapper);
    	actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    	actor->GetProperty()->SetOpacity(1);

    	return actor;
  	};

  	const double red[3] = {1,0,0};
  	const double blue[3] = {0,0,1};
	const double gray[3] = {0.5,0.5,0.5};

	// Get user parameters
	grainAmount = ui->GrainCount_spinBox->value();
	grainSize = ui->GrainSize_doubleSpinBox->value();
	grainDistribution = ui->GrainDistribution_doubleSpinBox->value();
	targetPorosity = ui->TargetPorosity_doubleSpinBox->value();

	// Get simulation parameters
	overlapTolerance = startOverlapTolerance;
	growthFactor = startGrowthFactor;
	shrinkFactor = startShrinkFactor;
	sequenceCount = 0;

	// Initialize boxes containing spheres to allow for neighbor collision checks only
	double root = std::cbrt(static_cast<double>(grainAmount));  
	boxesSize = static_cast<int>(std::floor(root)); // size = floor( 3sqrt(objectCount) )
	boxes = std::vector<std::vector<std::vector<std::array<int,25>>>>(boxesSize, std::vector<std::vector<std::array<int,25>>>(boxesSize, std::vector<std::array<int,25>>(boxesSize, std::array<int,25>())));

	// Initialize objects
	centers.resize(grainAmount);
	centers = generateRandomPoints(grainAmount);
  	spheres.resize(grainAmount);
  	actors.resize(grainAmount);
	directions.resize(grainAmount);

	// Initialize spheres
  	for (int i = 0; i < grainAmount; ++i) {
    	spheres[i] = vtkSmartPointer<vtkSphere>::New();
    	spheres[i]->SetCenter(centers[i][0], centers[i][1], centers[i][2]);
		if (i <= (grainAmount / 2)) {
			spheres[i]->SetRadius(grainSize);
		} else {
			spheres[i]->SetRadius(grainSize * grainDistribution);
		}
		actors[i] = makeActor(spheres[i], grainSize, gray);
		mRenderer->AddActor(actors[i]);
		double boundingCubeSizex = upperBoundx - lowerBoundx;
		double cellSizex = boundingCubeSizex / boxesSize;
		double boundingCubeSizey = upperBoundy - lowerBoundy;
		double cellSizey = boundingCubeSizey / boxesSize;
		double boundingCubeSizez = upperBoundz - lowerBoundz;
		double cellSizez = boundingCubeSizez / boxesSize;
		int bx = static_cast<int>(std::floor((centers[i][0] - lowerBoundx) / cellSizex));
		int by = static_cast<int>(std::floor((centers[i][1] - lowerBoundy) / cellSizey));
		int bz = static_cast<int>(std::floor((centers[i][2] - lowerBoundz) / cellSizez));
		if (boxes[bx][by][bz][0] >= 9) {
			 std::cerr << "Error: More than 10 objects in a single box cell";
		} else {
			boxes[bx][by][bz][0]++;
			boxes[bx][by][bz][boxes[bx][by][bz][0]] = i;
		}
	}

	// Initialize Bounding Cube
	boundingCube = vtkSmartPointer<vtkCubeSource>::New();
	boundingCube->SetCenter(0, 0, 0);
	boundingCube->SetBounds(lowerBoundx, upperBoundx, lowerBoundy, upperBoundy, lowerBoundz, upperBoundz);
	boundingCube->Update();

	auto cubeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	cubeMapper->SetInputConnection(boundingCube->GetOutputPort());

	boundingCubeActor = vtkSmartPointer<vtkActor>::New();
	boundingCubeActor->SetMapper(cubeMapper);
	boundingCubeActor->GetProperty()->SetOpacity(0.2);
	//boundingCubeActor->GetProperty()->SetRepresentationToWireframe(); // just drawing the edges
	boundingCubeActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
	mRenderer->AddActor(boundingCubeActor);

  	mRenderer->ResetCamera();
}

// Bounds x-coordinate of a sphere inside of the bounding box
double MainWindow::boundSphereX(double coord, double radius) {
	if (coord > upperBoundx - radius) {
		coord = upperBoundx - radius;
	} else if (coord < lowerBoundx + radius) {
		coord = lowerBoundx + radius;
	}
	return coord;
}

// Bounds y-coordinate of a sphere inside of the bounding box
double MainWindow::boundSphereY(double coord, double radius) {
	if (coord > upperBoundy - radius) {
		coord = upperBoundy - radius;
	} else if (coord < lowerBoundy + radius) {
		coord = lowerBoundy + radius;
	}
	return coord;
}

// Bounds z-coordinate of a sphere inside of the bounding box
double MainWindow::boundSphereZ(double coord, double radius) {
	if (coord > upperBoundz - radius) {
		coord = upperBoundz - radius;
	} else if (coord < lowerBoundz + radius) {
		coord = lowerBoundz + radius;
	}
	return coord;
}

// Decreases size of all spheres according to shrinkFactor
void MainWindow::decreaseSize() {
	grainSize *= (1 - shrinkFactor);
	for (int i = 0; i < grainAmount; ++i) {
    	if (i <= (grainAmount / 2)) {
			spheres[i]->SetRadius(grainSize);
		} else {
			spheres[i]->SetRadius(grainSize * grainDistribution);
		}
	}
}

// Increases size of all spheres according to GrowthFactor
void MainWindow::increaseSize() {
	grainSize *= (1 + growthFactor);
	for (int i = 0; i < grainAmount; ++i) {
    	if (i <= (grainAmount / 2)) {
			spheres[i]->SetRadius(grainSize);
		} else {
			spheres[i]->SetRadius(grainSize * grainDistribution);
		}
		attemptMove(i, 1, 0, 1, 0, 0, 0);
	}
}

// Check if the objects collide and return penetration depth
double MainWindow::checkCollision(int id1, int id2)
{
  	double distance = vtkMath::Distance2BetweenPoints(spheres[id1]->GetCenter(), spheres[id2]->GetCenter());
	
  	double radii = spheres[id1]->GetRadius() + spheres[id2]->GetRadius();
	
	if (distance < radii * radii) {
		double distance2 = std::sqrt(distance);
		// actors[id1]->GetProperty()->SetColor(0.0, 1.0, 1.0);
		// actors[id2]->GetProperty()->SetColor(0.0, 1.0, 1.0);
		return (radii - distance2);
	} else {
		return 0; // No collision
	}
}

// Attempt to move a sphere away from overlapping spheres
void MainWindow::attemptMove(int i, double maxOverlap, double totalOverlap, double collisions, double dx, double dy, double dz) {
	double x = spheres[i]->GetCenter()[0];
	double y = spheres[i]->GetCenter()[1];
	double z = spheres[i]->GetCenter()[2];
	double radius = spheres[i]->GetRadius();
	bool moving = true;
	double factor = 1;
	double shift = (totalOverlap / collisions) * factor;
	double xNew = boundSphereX(x + shift * dx, radius);
	double yNew = boundSphereY(y + shift * dy, radius);
	double zNew = boundSphereZ(z + shift * dz, radius);
	spheres[i]->SetCenter(xNew, yNew, zNew);
}

// If a sphere has a bigger overlap than tolerated, attempt to move it
void MainWindow::moveSpheres() {
	for (int bx = 0; bx < boxesSize; ++bx) {
		for (int by = 0; by < boxesSize; ++by) {
			for (int bz = 0; bz < boxesSize; ++bz) {
				for (int i = 1; i <= boxes[bx][by][bz][0]; ++i) {
					int sphere = boxes[bx][by][bz][i];
					double overlapRatio = directions[sphere][5] / spheres[sphere]->GetRadius();
					if (overlapRatio > overlapTolerance && directions[sphere][4] > 0) {
						attemptMove(sphere, directions[sphere][5], directions[sphere][3], directions[sphere][4], directions[sphere][0], directions[sphere][1], directions[sphere][2]);
						// Change boxes if moved into a different box
						double boundingCubeSizex = upperBoundx - lowerBoundx;
						double cellSizex = boundingCubeSizex / boxesSize;
						double boundingCubeSizey = upperBoundy - lowerBoundy;
						double cellSizey = boundingCubeSizey / boxesSize;
						double boundingCubeSizez = upperBoundz - lowerBoundz;
						double cellSizez = boundingCubeSizez / boxesSize;
						int x = static_cast<int>(std::floor((spheres[sphere]->GetCenter()[0] - lowerBoundx) / cellSizex));
						int y = static_cast<int>(std::floor((spheres[sphere]->GetCenter()[1] - lowerBoundy) / cellSizey));
						int z = static_cast<int>(std::floor((spheres[sphere]->GetCenter()[2] - lowerBoundz) / cellSizez));
						if (x != bx || y != by || z != bz) {
							// Remove object from box
							for (int j = i; j < boxes[bx][by][bz][0]; ++j) {
								boxes[bx][by][bz][j] = boxes[bx][by][bz][j+1];
							}
							boxes[bx][by][bz][boxes[bx][by][bz][0]] = 0;
							boxes[bx][by][bz][0]--;
							// Add object to box
							if (boxes[x][y][z][0] >= 9) {
			 					std::cerr << "Error: More than 10 objects in a single box cell";
							} else {
								boxes[x][y][z][0]++;
								boxes[x][y][z][boxes[x][y][z][0]] = sphere;
							}
						}
						
					}
				}
			}
		}
	}
}

// Check collision between a sphere and all spheres in neighboring boxes
void MainWindow::findOverlapsNeighbors(int sphere, int bx, int by, int bz) {
	for (int nbx = (bx > 0 ? bx - 1 : bx); nbx <= (bx + 1 < boxesSize ? bx + 1 : bx); ++nbx) {
		for (int nby = (by > 0 ? by - 1 : by); nby <= (by + 1 < boxesSize ? by + 1 : by); ++nby) {
			for (int nbz = (bz > 0 ? bz - 1 : bz); nbz <= (bz + 1 < boxesSize ? bz + 1 : bz); ++nbz) {
				// Check all objects (neighbors) inside a neighboring box
				for (int i = 1; i <= boxes[nbx][nby][nbz][0]; ++i) {
					int neighbor = boxes[nbx][nby][nbz][i];
					if (nbx != bx || nby != by || nbz != bz || neighbor != sphere) { // Do not check collision with itself
						double overlap = checkCollision(sphere, neighbor);
    					if (overlap > 0) { // Add vector indicating movement away from colliding sphere
							double distance = vtkMath::Distance2BetweenPoints(spheres[sphere]->GetCenter(), spheres[neighbor]->GetCenter());
							distance = std::sqrt(distance);
							if (distance < 0.0001) {
								std::random_device rd;
								std::mt19937 gen(rd());
								std::normal_distribution<> dist(0.0, 1.0);
								double x = dist(gen);
								double y = dist(gen);
								double z = dist(gen);
								double norm = std::sqrt(x*x + y*y + z*z);
								double radius = std::min(spheres[sphere]->GetRadius(), spheres[neighbor]->GetRadius());
								directions[sphere][0] += radius * x / norm;
								directions[sphere][1] += radius * y / norm;
								directions[sphere][2] += radius * z / norm;
							} else {
								double ux = (spheres[sphere]->GetCenter()[0] - spheres[neighbor]->GetCenter()[0]) / distance;
								double uy = (spheres[sphere]->GetCenter()[1] - spheres[neighbor]->GetCenter()[1]) / distance;
								double uz = (spheres[sphere]->GetCenter()[2] - spheres[neighbor]->GetCenter()[2]) / distance;
								directions[sphere][0] += ux;
								directions[sphere][1] += uy;
								directions[sphere][2] += uz;
							}
							directions[sphere][3] += overlap; // total overlap
							directions[sphere][4]++; // collision count
							if (overlap > directions[sphere][5]) { // biggest overlap
								directions[sphere][5] = overlap;
							}
						}
					}
				}
			}
		}
	}
}


// Check collision between every pair of spheres and find their total overlap
bool MainWindow::findOverlaps() {
	for (auto & slot : directions) {
    	slot.fill(0.0);
	}
	for (int bx = 0; bx < boxesSize; ++bx) {
		for (int by = 0; by < boxesSize; ++by) {
			for (int bz = 0; bz < boxesSize; ++bz) {
				for (int i = 1; i <= boxes[bx][by][bz][0]; ++i) {
					int sphere = boxes[bx][by][bz][i];
					findOverlapsNeighbors(sphere, bx, by, bz);
				}
			}
		}
	}

	for (int i = 0; i < grainAmount; ++i) {
		double overlapRatio = directions[i][5] / spheres[i]->GetRadius();
		if (overlapRatio > overlapTolerance) {
			return true;
		}
	}
	return false;
}

// Returns the current porosity (fraction of empty space)
double MainWindow::getTruePorosity() {
	auto grainUnion = vtkSmartPointer<vtkImplicitBoolean>::New();
	grainUnion->SetOperationTypeToUnion();
	for (auto shape : spheres) {
		grainUnion->AddFunction(shape);
	}
    	
	// Sample a 3d grid
	auto sampler = vtkSmartPointer<vtkSampleFunction>::New();
	sampler->SetImplicitFunction(grainUnion);
	sampler->SetModelBounds(lowerBoundx, upperBoundx, lowerBoundy, upperBoundy, lowerBoundz, upperBoundz);
	sampler->SetSampleDimensions(100, 100, 100);  // increase for more accuracy
	sampler->ComputeNormalsOff();

	auto contour = vtkSmartPointer<vtkContourFilter>::New();
	contour->SetInputConnection(sampler->GetOutputPort());
	contour->SetValue(0, 0.0);
	contour->Update();

	auto triFilter = vtkSmartPointer<vtkTriangleFilter>::New();
	triFilter->SetInputConnection(contour->GetOutputPort());

	auto massProps = vtkSmartPointer<vtkMassProperties>::New();
	massProps->SetInputConnection(triFilter->GetOutputPort());
	massProps->Update();

	double volume = massProps->GetVolume();
	double totalVolume = (upperBoundx - lowerBoundx) * (upperBoundy - lowerBoundy) * (upperBoundz - lowerBoundz);
	double porosity = 1 - (volume / totalVolume);
	return porosity;
} 

// Returns the current porosity (fraction of empty space)
double MainWindow::getSamplePorosity() {
    auto grainUnion = vtkSmartPointer<vtkImplicitBoolean>::New();
    grainUnion->SetOperationTypeToUnion();
    for (auto& shape : spheres) {
        grainUnion->AddFunction(shape);
    }

    auto sampler = vtkSmartPointer<vtkSampleFunction>::New();
    sampler->SetImplicitFunction(grainUnion);
    sampler->SetModelBounds(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);
    sampler->SetSampleDimensions(100, 100, 100); // increase for more accuracy
    sampler->ComputeNormalsOff();

    auto contour = vtkSmartPointer<vtkContourFilter>::New();
    contour->SetInputConnection(sampler->GetOutputPort());
    contour->SetValue(0, 0.0);
	contour->Update();

	vtkSmartPointer<vtkPolyData> polyData = contour->GetOutput();
    if (!polyData || polyData->GetNumberOfCells() == 0) {
        return 1.0; // No geometry found means 100% porous
    }

    auto triFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triFilter->SetInputConnection(contour->GetOutputPort());

    auto massProps = vtkSmartPointer<vtkMassProperties>::New();
    massProps->SetInputConnection(triFilter->GetOutputPort());
    massProps->Update();

    double solidVolume = massProps->GetVolume();
    double porosity = 1.0 - (solidVolume / 1.0);
    return porosity;
} 

// Run one simulation step and render
void MainWindow::simulationStep() {
	if (findOverlaps()) {
		if (moveCount >= moveTolerance) {
			decreaseSize();
			moveCount = 0;
			decreaseCount++;
		} else {
			moveSpheres();
			moveCount++;
		}
	} else {
		increaseSize();
		moveCount = 0;
	}

	if (decreaseCount >= decreaseTolerance) {
		decreaseCount = 0;
		sequenceCount++;
		overlapTolerance *= 0.8;
		growthFactor *= 0.8;
		shrinkFactor *= 0.8;
	}
}

// Renders the current state of the simulator
void MainWindow::renderState() {
	currentPorosity = getTruePorosity();
	QString txt = QStringLiteral("%1%").arg(currentPorosity * 100.0, 0, 'f', 2);
	ui->TruePorosityValue_label->setText(txt);
	vtkCamera* cam = mRenderer->GetActiveCamera();
	cam->ParallelProjectionOff();
	mRenderer->ResetCamera();
	mRenderer->ResetCameraClippingRange();
	mRenderWindow->Render();
}

// Generates an initial setup based on input parameters
void MainWindow::onSimulateStartClick() {
	QElapsedTimer timer;
    timer.start();
	generateInitial();
	renderState();
	qint64 elapsed = timer.elapsed(); // milliseconds
	ui->TimerValue_label->setText(QString("%1 ms").arg(elapsed));
}

// Performs the next simulation step
void MainWindow::onSimulateStepClick() {
	QElapsedTimer timer;
    timer.start();
	simulationStep();
	renderState();
	qint64 elapsed = timer.elapsed(); // milliseconds
	ui->TimerValue_label->setText(QString("%1 ms").arg(elapsed));
}

// Runs a full simulation (only render at the end)
void MainWindow::onSimulateClick() {
	QElapsedTimer timer;
    timer.start();
	generateInitial();
	while(sequenceCount < sequenceMax) {
		simulationStep();
	}
	int i = 0;
	// Minimize overlap without changing size
	while(findOverlaps() && i < 10) {
		moveSpheres();
		i++;
	}
	currentPorosity = getTruePorosity();
	shrinkFactor = 0.01; // Small shrink factor to approach target porosity accurately
	while (currentPorosity < targetPorosity) {
		decreaseSize();
		currentPorosity = getTruePorosity();
	}
	renderState();
	qint64 elapsed = timer.elapsed(); // milliseconds
	ui->TimerValue_label->setText(QString("%1 ms").arg(elapsed));
}

// returns a QImage of the last rendered frame and saves it as an image
void MainWindow::saveScreenshot(const QString& filename)
{
    QImage img = ui->openGLWidget->grabFramebuffer();
    img.save(filename);
}




