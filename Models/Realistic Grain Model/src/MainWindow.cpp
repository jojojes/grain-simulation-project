#include <MainWindow.h>

#include "ui_MainWindow.h"

#include "ImplicitGrain.h"

#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkContourFilter.h>
#include <vtkCubeSource.h>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkDelaunay2D.h>
#include <vtkFlyingEdges3D.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkPolyLine.h>
#include <vtkPoints.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRuledSurfaceFilter.h>
#include <vtkSampleFunction.h>
#include <vtkSphere.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkImplicitBoolean.h>
#include <vtkImplicitFunction.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkImplicitSelectionLoop.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkMassProperties.h>
#include <vtkMath.h>

#include <QMessageBox>
#include <QFileDialog>

#include <random>
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

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
    ui->GrainCount_spinBox->setValue(grainCount);

	ui->GrainSize_doubleSpinBox->setRange(0.01, 100.0);
    ui->GrainSize_doubleSpinBox->setSingleStep(0.01);
    ui->GrainSize_doubleSpinBox->setValue(grainSize);

	ui->GrainLength_doubleSpinBox->setRange(0.01, 5.0);
    ui->GrainLength_doubleSpinBox->setSingleStep(0.01);
    ui->GrainLength_doubleSpinBox->setValue(grainLength);

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
	QObject::connect(ui->Simulate_button, &QPushButton::clicked, this, &MainWindow::onSimulateClick);
}

MainWindow::~MainWindow() {
    delete ui;
}

// Generates a random point within the bounds
std::vector<std::array<double,3>> MainWindow::generateRandomPoints(int n)
{
    std::random_device random;
    std::mt19937 generator(random());
    std::uniform_real_distribution<double> distributionx(lowerBoundx + grainSize, upperBoundx - grainSize);
	std::uniform_real_distribution<double> distributiony(lowerBoundy + grainSize, upperBoundy - grainSize);
    std::uniform_real_distribution<double> distributionz(lowerBoundz + grainSize, upperBoundz - grainSize);
    std::vector<std::array<double,3>> coords;
    coords.reserve(n);
    for (int i = 0; i < n; ++i) {
        coords.push_back({distributionx(generator), distributiony(generator), distributionz(generator)});
    }
    return coords;
}

// Returns random coordinates in given bounds
std::vector<std::array<double,3>> MainWindow::generateRandomRotation(int n)
{
    std::random_device random;
    std::mt19937 generator(random());
    std::uniform_real_distribution<double> distribution(0, 360);

    std::vector<std::array<double,3>> angles;
    angles.reserve(n);
    for (int i = 0; i < n; ++i) {
        angles.push_back({ distribution(generator), distribution(generator), distribution(generator) });
    }
    return angles;
}

void MainWindow::generateInitial() {
	mRenderer->RemoveAllViewProps();
	grains.clear();
  	actors.clear();
  	directions.clear();

	auto makeActor = [&](vtkImplicitFunction* implicit, int i, const double color[3]) {
		vtkNew<vtkSampleFunction> sampler;
		sampler->SetImplicitFunction(implicit);
		double pad = 0.1;
		sampler->SetModelBounds(lowerBoundx, upperBoundx, lowerBoundy, upperBoundy, lowerBoundz, upperBoundz);
		sampler->SetSampleDimensions(150, 150, 150); // Sampling resolution
		sampler->ComputeNormalsOff();
		sampler->Update();

		vtkNew<vtkContourFilter> contourFilter;
		contourFilter->SetInputConnection(sampler->GetOutputPort());
		contourFilter->SetValue(0, 0.0);
		contourFilter->Update();

		transforms[i] = vtkSmartPointer<vtkTransform>::New();
		transforms[i]->Translate(centers[i][0], centers[i][1], centers[i][2]);

		vtkNew<vtkTransformFilter> xfFilter2;
		xfFilter2->SetTransform(transforms[i]); // identity
		xfFilter2->SetInputConnection(contourFilter->GetOutputPort());
		xfFilter2->Update();

		vtkNew<vtkPolyDataMapper> mapper;
		mapper->SetInputConnection(xfFilter2->GetOutputPort());
		mapper->ScalarVisibilityOff();

		vtkNew<vtkActor> actor;
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(color[0], color[1], color[2]);
		actor->GetProperty()->SetOpacity(1);

		return actor;
	};

	const double red[3] = {1,0,0};
  	const double blue[3] = {0,0,1};
	const double green[3] = {0,1,0};
	const double white[3] = {1,1,1};
	const double gray[3] = {0.5,0.5,0.5};

	// Get user parameters
	grainCount = ui->GrainCount_spinBox->value();
	grainSize = ui->GrainSize_doubleSpinBox->value();
	grainLength = ui->GrainLength_doubleSpinBox->value();
	grainDistribution = ui->GrainDistribution_doubleSpinBox->value();
	targetPorosity = ui->TargetPorosity_doubleSpinBox->value();

	// Get simulation parameters
	overlapTolerance = startOverlapTolerance;
	growthFactor = startGrowthFactor;
	shrinkFactor = startShrinkFactor;
	sequenceCount = 0;

	// Initialize objects
	centers.resize(grainCount);
	centers = generateRandomPoints(grainCount);
	rotations.resize(grainCount);
	rotations = generateRandomRotation(grainCount);
  	grains.resize(grainCount);
	transforms.resize(grainCount);
  	actors.resize(grainCount);
	directions.resize(grainCount);

	// Initialize grains
  	for (int i = 0; i < grainCount; ++i) {
		auto implicit2D = getContourImplicit();
		grains[i] = vtkSmartPointer<ImplicitGrain>::New();
		grains[i]->SetImplicit2D(implicit2D);
		grains[i]->SetA(grainSize);
  		grains[i]->SetB(grainSize);
  		grains[i]->SetC(grainSize * grainLength);
		//grains[i]->SetCenter(centers[i][0], centers[i][1], centers[i][2]);
		//grains[i]->SetRotation(rotations[i][0], rotations[i][1], rotations[i][2]);
		actors[i] = makeActor(grains[i], i, gray);
		mRenderer->AddActor(actors[i]);
	}

	//Initialize Bounding Cube
	boundingCube = vtkSmartPointer<vtkCubeSource>::New();
	boundingCube->SetCenter(0, 0, 0);
	boundingCube->SetBounds(lowerBoundx, upperBoundx, lowerBoundy, upperBoundy, lowerBoundz, upperBoundz);
	boundingCube->Update();

	auto cubeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	cubeMapper->SetInputConnection(boundingCube->GetOutputPort());

	auto cubeActor = vtkSmartPointer<vtkActor>::New();
	cubeActor->SetMapper(cubeMapper);
	cubeActor->GetProperty()->SetOpacity(0.0);
	//cubeActor->GetProperty()->SetRepresentationToWireframe(); // Draw just the edges
	cubeActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
	mRenderer->AddActor(cubeActor);

  	mRenderer->ResetCamera();
	mRenderWindow->Render();
}

vtkSmartPointer<vtkImplicitPolyDataDistance> MainWindow::getContourImplicit() {
	vtkNew<vtkImplicitPolyDataDistance> implicit2D;
	// Ask the user for a CSV file
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Open Contour CSV"),
        QString(),
        tr("CSV Files (*.csv);;All Files (*)")
    );
    if (fileName.isEmpty()) { return implicit2D; } 


    // Read all lines of the CSV into rawPoints
    std::vector<std::array<double,2>> rawPoints;
    {
        std::ifstream in(fileName.toStdString());
        if (!in.is_open())
        {
            QMessageBox::warning(this, tr("File Error"), tr("Could not open file:\n%1").arg(fileName));
            return implicit2D;
        }
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty()) { 
				continue; 
			}
            std::stringstream ss(line);
            std::string sx, sy;
            if (!std::getline(ss, sx, ',')) {
				continue;
			}
            if (!std::getline(ss, sy, ',')) {
				continue;
			}
            try {
                double x = std::stod(sx);
                double y = std::stod(sy);
                rawPoints.push_back({ x, y });
            }
            catch (...) {
                // skip invalid lines
            }
        }
        in.close();
        if (rawPoints.empty()) {
            QMessageBox::warning(this, tr("CSV Error"), tr("No valid (x,y) points were found in:\n%1").arg(fileName));
            return implicit2D;
        }
    }

    // Compute centroid and sort by atan2
    double cx = 0.0, cy = 0.0;
    for (auto &p : rawPoints) { cx += p[0]; cy += p[1]; }
    cx /= rawPoints.size();
    cy /= rawPoints.size();
    std::sort(rawPoints.begin(), rawPoints.end(), [&](auto &a, auto &b) {
        double aA = std::atan2(a[1]-cy, a[0]-cx);
        double aB = std::atan2(b[1]-cy, b[0]-cx);
        return aA < aB;
    });


    // Find bounding box of rawPoints to normalize into [-grainHalfSize, +grainHalfSize].
    double minX = rawPoints[0][0], maxX = rawPoints[0][0];
    double minY = rawPoints[0][1], maxY = rawPoints[0][1];
    for (auto &p : rawPoints)
    {
        minX = std::min(minX, p[0]);
        maxX = std::max(maxX, p[0]);
        minY = std::min(minY, p[1]);
        maxY = std::max(maxY, p[1]);
    }
    double width  = maxX - minX;
    double height = maxY - minY;
    double maxDim = std::max(width, height);

    double grainHalfSize = 1.0;

    vtkNew<vtkPoints>  points2D;
    vtkNew<vtkPolygon> polygon2D;
    int N = static_cast<int>(rawPoints.size());
    polygon2D->GetPointIds()->SetNumberOfIds(N);

    double rawCenterX = minX + 0.5 * width;
    double rawCenterY = minY + 0.5 * height;

    for (int i = 0; i < N; ++i)
    {
        double rx = rawPoints[i][0];
        double ry = rawPoints[i][1];
        double normX = (rx - rawCenterX) * (grainHalfSize / (0.5 * maxDim));
        double normY = (ry - rawCenterY) * (grainHalfSize / (0.5 * maxDim));
        points2D->InsertNextPoint(normX, normY, 0.0);
        polygon2D->GetPointIds()->SetId(i, i);
    }

    vtkNew<vtkCellArray> polys2D;
    polys2D->InsertNextCell(polygon2D);

    vtkNew<vtkPolyData> polyData2D;
    polyData2D->SetPoints(points2D);
    polyData2D->SetPolys(polys2D);

    vtkNew<vtkCleanPolyData> clean2D;
    clean2D->SetInputData(polyData2D);
    clean2D->SetTolerance(1e-6);
    clean2D->Update();

	// Transform grainHalfSize contour to a "unit contour" that fits in [-1,1] in X-axis and Y-axis
    vtkNew<vtkTransform> norm2DTransform;
    vtkNew<vtkTransformPolyDataFilter> norm2DFilter;
    norm2DTransform->Identity();
    norm2DTransform->Scale(1.0 / grainHalfSize, 1.0 / grainHalfSize, 1.0);

    norm2DFilter->SetTransform(norm2DTransform);
    norm2DFilter->SetInputConnection(clean2D->GetOutputPort());
    norm2DFilter->Update();

    vtkSmartPointer<vtkPolyData> unitContour2D = vtkSmartPointer<vtkPolyData>::New();
    unitContour2D->ShallowCopy(norm2DFilter->GetOutput());

	// Create and configure ImplicitGrain using vtkImplicitPolyDataDistance based on unitContour2D
    
    implicit2D->SetInput(unitContour2D);
	return implicit2D;
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
	for (int i = 0; i < grainCount; ++i) {
    	if (i <= (grainCount / 2)) {
			grains[i]->SetA(grainSize);
  			grains[i]->SetB(grainSize);
  			grains[i]->SetC(grainSize * grainLength);
		} else {
			grains[i]->SetA(grainSize);
  			grains[i]->SetB(grainSize);
  			grains[i]->SetC(grainSize * grainLength);
		}
	}
}

// Increases size of all spheres according to GrowthFactor
void MainWindow::increaseSize() {
	grainSize *= (1 + growthFactor);
	for (int i = 0; i < grainCount; ++i) {
    	if (i <= (grainCount / 2)) {
			grains[i]->SetA(grainSize);
  			grains[i]->SetB(grainSize);
  			grains[i]->SetC(grainSize * grainLength);
		} else {
			grains[i]->SetA(grainSize);
  			grains[i]->SetB(grainSize);
  			grains[i]->SetC(grainSize * grainLength);
		}
		attemptMove(i, 1, 0, 1, 0, 0, 0);
	}
}

// Check if the objects collide and return penetration depth
double MainWindow::checkCollision(int id1, int id2) {
  	//double distance = vtkMath::Distance2BetweenPoints(grains[id1]->GetCenter(), grains[id2]->GetCenter());
	double distance = vtkMath::Distance2BetweenPoints(centers[id1], centers[id2]);
  	double radii = grains[id1]->GetRadius() + grains[id2]->GetRadius();
	if (distance < radii * radii) {
		actors[id1]->GetProperty()->SetColor(0.0, 1.0, 1.0);
		actors[id2]->GetProperty()->SetColor(0.0, 1.0, 1.0);
		return (radii - std::sqrt(distance));
	} else {
		return 0; // No collision
	}
}

// Attempt to move a sphere away from overlapping spheres
void MainWindow::attemptMove(int i, double maxOverlap, double totalOverlap, double collisions, double dx, double dy, double dz) {
	double x = centers[i][0];
	double y = centers[i][1];
	double z = centers[i][2];
	double radius = grains[i]->GetRadius();
	double factor = 1;
	double shift = (totalOverlap / collisions) * factor;
	double xNew = boundSphereX(x + shift * dx, radius);
	double yNew = boundSphereY(y + shift * dy, radius);
	double zNew = boundSphereZ(z + shift * dz, radius);
	//grains[i]->SetCenter(xNew, yNew, zNew);
	centers[i][0] = xNew;
	centers[i][1] = yNew;
	centers[i][2] = zNew;
	transforms[i]->Identity();
	transforms[i]->Translate(xNew, yNew, zNew);
}

// If a sphere has a bigger overlap than tolerated, attempt to move it
void MainWindow::moveGrains() {
	for (int i = 0; i < grainCount; ++i) {
		double overlapRatio = directions[i][5] / grains[i]->GetRadius();
		if (overlapRatio > overlapTolerance) {
			attemptMove(i, directions[i][5], directions[i][3], directions[i][4], directions[i][0], directions[i][1], directions[i][2]);
		}
	}
}


// Check collision between every pair of grains and find their total overlap
bool MainWindow::findOverlaps() {
	for (auto & slot : directions) {
    	slot.fill(0.0);
	}
	for (int i = 0; i < grainCount; ++i) {
		for (int j = 0; j < grainCount; ++j) {
			if (j != i) { // Do not check collision with itself
				double overlap = checkCollision(i, j);
    			if (overlap > 0) { // Add vector indicating movement away from colliding grain
					//double distance = vtkMath::Distance2BetweenPoints(grains[i]->GetCenter(), grains[j]->GetCenter());
					double distance = vtkMath::Distance2BetweenPoints(centers[i], centers[i]);
					distance = std::sqrt(distance);
					double ux = (centers[i][0] - centers[j][0]) / distance;
					double uy = (centers[i][1] - centers[j][1]) / distance;
					double uz = (centers[i][2] - centers[j][2]) / distance;
					directions[i][0] += ux;
					directions[i][1] += uy;
					directions[i][2] += uz;
					directions[i][3] += overlap; // total overlap
					directions[i][4]++; // collision count
					if (overlap > directions[i][5]) { // biggest overlap
						directions[i][5] = overlap;
					}
				}
			}
		}
	}

	for (int i = 0; i < grainCount; ++i) {
		double overlapRatio = directions[i][5] / grains[i]->GetRadius();
		if (overlapRatio > overlapTolerance) {
			return true;
		}
	}
	return false;
}

// Returns the current porosity (fraction of empty space)
double MainWindow::getPorosity() {
	auto unionFunc = vtkSmartPointer<vtkImplicitBoolean>::New();
	unionFunc->SetOperationTypeToUnion();
	for (auto shape : grains) {
		unionFunc->AddFunction(shape);
	}
    	
	// Sample a 3d grid
	auto sampler = vtkSmartPointer<vtkSampleFunction>::New();
	sampler->SetImplicitFunction(unionFunc);
	sampler->SetModelBounds(lowerBoundx, upperBoundx, lowerBoundy, upperBoundy, lowerBoundz, upperBoundz);
	sampler->SetSampleDimensions(100, 100, 100);  // increase for more accuracy
	sampler->ComputeNormalsOff();
	sampler->Update();

	auto contour = vtkSmartPointer<vtkContourFilter>::New();
	contour->SetInputConnection(sampler->GetOutputPort());
	contour->SetValue(0, 0.0);

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

void MainWindow::simulationStep() {
	if (findOverlaps()) {
		if (moveCount >= moveTolerance) {
			decreaseSize();
			moveCount = 0;
			decreaseCount++;
		} else {
			moveGrains();
			moveCount++;
		}
	} else {
		increaseSize();
		moveCount = 0;
	}

	if (decreaseCount >= decreaseTolerance) {
		decreaseCount = 0;
		sequenceCount++;
		overlapTolerance *= 0.5;
		growthFactor *= 0.5;
		shrinkFactor *= 0.5;
	}
}

// Renders the current state of the simulator
void MainWindow::renderState() {
	//currentPorosity = getPorosity();
	//QString txt = QStringLiteral("%1%").arg(currentPorosity * 100.0, 0, 'f', 2);
	//ui->TruePorosityValue_label->setText(txt);
	mRenderWindow->Render();
}

// Generates an initial setup based on input parameters
void MainWindow::onSimulateStartClick()
{
	QElapsedTimer timer;
    timer.start();
	generateInitial();
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
	qint64 elapsed = timer.elapsed(); // milliseconds
	ui->TimerValue_label->setText(QString("%1 ms").arg(elapsed));
}


