#include "Canvas.h"
#include <QPainter>
#include <iostream>
#include <QFileInfoList>
#include <QDir>
#include <QMessageBox>
#include <QTextStream>
#include <QDomDocument>
#include <QResizeEvent>
#include <QtWidgets/QApplication>
#include <QDate>
#include <glm/gtx/string_cast.hpp>
#include "MainWindow.h"
#include "Rectangle.h"
#include "Circle.h"
#include "Polygon.h"

namespace canvas {

	Canvas::Canvas(MainWindow* mainWin) : QWidget((QWidget*)mainWin) {
		this->mainWin = mainWin;
		ctrlPressed = false;
		shiftPressed = false;

		origin = QPoint(width() * 0.5, height() * 0.5);
		scale = 10.0;

		mode = MODE_SELECT;
		operation.reset();
		current_shape.reset();

		// add an empty design to the history as an initial state
		history.push(design);

		animation_timer = NULL;
		collision_check = true;
		restrict_motion_range = true;
		show_solutions = false;
		show_grid_lines = true;
		show_input_poses = true;
		show_linkage = true;

		// parameters for sampling and particle filter
		num_samples = 10000;
		stddev_position = 0.8;
		stddev_orientation = 0.04;
		avoid_branch_defect = true;
		min_transmission_angle = 0.15;
		weights = { 1, 10, 1, 1 };
		num_particles = 100;
		num_pf_iterations = 20;
		record_pf = false;

		selectedJoint = std::make_pair(-1, -1);
		linkage_type = LINKAGE_WATTI;
	}

	Canvas::~Canvas() {
	}

	void Canvas::clear() {
		design.clear();
		selected_shape.reset();

		// select 1st layer
		setLayer(0);

		// clear the kinematic data
		kinematics.clear();
		solutions.clear();
		selectedJoint = std::make_pair(-1, -1);

		update();
	}

	void Canvas::selectAll() {
		design.selectAll();
		mode = MODE_SELECT;
		update();
	}

	void Canvas::unselectAll() {
		design.unselectAll();
		current_shape.reset();
		update();
	}

	void Canvas::deleteSelectedShapes() {
		history.push(design);

		design.deleteSelectedShapes();

		current_shape.reset();
		update();
	}

	void Canvas::undo() {
		try {
			design = history.undo();
			update();
		}
		catch (char* ex) {
		}
	}

	void Canvas::redo() {
		try {
			design = history.redo();
			update();
		}
		catch (char* ex) {
		}
	}

	void Canvas::copySelectedShapes() {
		design.copySelectedShapes();
	}

	void Canvas::pasteCopiedShapes() {
		design.pasteCopiedShapes();
		current_shape.reset();
		mode = MODE_SELECT;
		update();
	}

	void Canvas::setMode(int mode) {
		if (this->mode != mode) {
			this->mode = mode;

			// clear
			unselectAll();
			selectedJoint = std::make_pair(-1, -1);

			update();
		}
	}
	
	void Canvas::addLayer() {
		design.addLayer();
		setLayer(design.num_layers - 1);
	}

	void Canvas::insertLayer() {
		design.insertLayer();
		setLayer(design.layer_id);
	}

	void Canvas::deleteLayer() {
		if (design.deleteLayer()) {
			setLayer(design.layer_id);
		}
	}

	void Canvas::setLayer(int layer_id) {
		design.unselectAll();
		design.layer_id = layer_id;
		current_shape.reset();

		// change the mode to SELECT
		setMode(MODE_SELECT);

		update();
	}

	void Canvas::open(const QString& filename) {
		QFile file(filename);
		if (!file.open(QFile::ReadOnly | QFile::Text)) throw "File cannot open.";

		// if the animation is running, stop it.
		if (animation_timer) {
			stop();
		}

		design.load(filename);
		history.push(design);

		// no currently drawing shape
		current_shape.reset();

		mode = MODE_SELECT;

		// clear the kinematic data
		kinematics.clear();
		solutions.clear();

		// update the layer menu based on the loaded data
		mainWin->initLayerMenu(design.num_layers);

		update();
	}

	void Canvas::save(const QString& filename) {
		design.save(filename);
	}

	void Canvas::run() {
		if (animation_timer == NULL) {
			animation_timer = new QTimer(this);
			connect(animation_timer, SIGNAL(timeout()), this, SLOT(animation_update()));
			animation_timer->start(10);
		}
	}

	void Canvas::stop() {
		if (animation_timer != NULL) {
			animation_timer->stop();
			delete animation_timer;
			animation_timer = NULL;
		}
	}

	void Canvas::speedUp() {
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].speedUp();
		}
	}

	void Canvas::speedDown() {
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].speedDown();
		}
	}

	void Canvas::invertSpeed() {
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].invertSpeed();
		}
	}

	void Canvas::stepForward() {
		if (animation_timer == NULL) {
			for (int i = 0; i < kinematics.size(); i++) {
				try {
					kinematics[i].stepForward(collision_check);
				}
				catch (char* ex) {
					kinematics[i].invertSpeed();
					std::cerr << "Animation is stopped by error:" << std::endl;
					std::cerr << ex << std::endl;
				}
			}
			update();
		}
	}

	void Canvas::stepBackward() {
		if (animation_timer == NULL) {
			for (int i = 0; i < kinematics.size(); i++) {
				try {
					kinematics[i].stepBackward(collision_check);
				}
				catch (char* ex) {
					kinematics[i].invertSpeed();
					std::cerr << "Animation is stopped by error:" << std::endl;
					std::cerr << ex << std::endl;
				}
			}
			update();
		}
	}

	glm::dvec2 Canvas::screenToWorldCoordinates(const glm::dvec2& p) {
		return screenToWorldCoordinates(p.x, p.y);
	}

	glm::dvec2 Canvas::screenToWorldCoordinates(double x, double y) {
		return glm::dvec2((x - origin.x()) / scale, -(y - origin.y()) / scale);
	}

	glm::dvec2 Canvas::worldToScreenCoordinates(const glm::dvec2& p) {
		return glm::dvec2(origin.x() + p.x * scale, origin.y() - p.y * scale);
	}

	void Canvas::calculateSolutions(int linkage_type) {
		mainWin->ui.statusBar->showMessage("Please wait for a moment...");

		// change the mode to kinematics
		setMode(MODE_KINEMATICS);
		mainWin->ui.actionKinematics->setChecked(true);

		time_t start = clock();

		this->linkage_type = linkage_type;

		// get the geometry of fixed rigid bodies, moving bodies, linkage regions
		fixed_bodies.clear();
		moving_bodies.resize(design.moving_bodies.size());
		poses.resize(design.moving_bodies.size());
		linkage_region_pts.clear();
		linkage_avoidance_pts.clear();

		for (int i = 0; i < design.fixed_bodies.size(); i++) {
			fixed_bodies.push_back(kinematics::Object25D(design.fixed_bodies[i]->getPoints()));
		}
		for (int i = 0; i < design.moving_bodies.size(); i++) {
			poses[i].resize(design.moving_bodies[i].poses.size());

			moving_bodies[i] = kinematics::Object25D(design.moving_bodies[i].poses[0]->getPoints());

			// set pose matrices
			for (int j = 0; j < design.moving_bodies[i].poses.size(); j++) {
				poses[i][j] = design.moving_bodies[i].poses[j]->getModelMatrix();
			}

			if (design.moving_bodies[i].linkage_region) {
				linkage_region_pts.push_back(design.moving_bodies[i].linkage_region->getPoints());
			}
			else {
				// use a bounding box as a default linkage region
				canvas::BoundingBox bbox;
				for (int j = 0; j < design.fixed_bodies.size(); j++) {
					bbox.addPoints(design.fixed_bodies[j]->getPoints());
				}
				bbox.addPoints(design.moving_bodies[i].poses[0]->getPoints());
				linkage_region_pts.push_back({ bbox.minPt, glm::dvec2(bbox.minPt.x, bbox.maxPt.y), bbox.maxPt, glm::dvec2(bbox.maxPt.x, bbox.minPt.y) });
			}

			if (design.moving_bodies[i].linkage_avoidance) {
				linkage_avoidance_pts.push_back(design.moving_bodies[i].linkage_avoidance->getPoints());
			}
			else {
				linkage_avoidance_pts.push_back({});
			}
		}

		// merged fixed body
		merged_fixed_bodies.clear();
		std::vector<std::vector<glm::dvec2>> polygons(design.fixed_bodies.size());
		for (int i = 0; i < design.fixed_bodies.size(); i++) {
			polygons[i] = design.fixed_bodies[i]->getPoints();
		}
		polygons = kinematics::unionPolygon(polygons);
		for (int i = 0; i < polygons.size(); i++) {
			merged_fixed_bodies.push_back(kinematics::Object25D(polygons[i]));
		}

		kinematics.clear();
		synthesis.clear();
		synthesis.resize(1);

		synthesis[0] = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesisWattI(merged_fixed_bodies, { stddev_position, stddev_orientation }, avoid_branch_defect, min_transmission_angle, 1.0, weights));

		initial_solutions.clear();
		solutions.clear();
		{
			// calculate a distance mapt for the linkage region
			cv::Mat dist_map;
			kinematics::BBox dist_map_bbox;
			kinematics::LinkageSynthesis::createDistanceMapForLinkageRegion(linkage_region_pts[0], 5, dist_map_bbox, dist_map);

			// calculate the center of the valid regions
			kinematics::BBox bbox = kinematics::boundingBox(linkage_region_pts[0]);
			glm::dvec2 bbox_center = bbox.center();

			std::vector<glm::dvec2> enlarged_linkage_region_pts;

			int cnt = 0;
			for (int scale = 1; scale <= 3 && cnt == 0; scale++) {
				// calculate the enlarged linkage region for the sampling region
				enlarged_linkage_region_pts = kinematics::LinkageSynthesis::enlargePolygon(linkage_region_pts[0], bbox_center, scale);

				// calculate the bounding boxe of the valid regions
				kinematics::BBox enlarged_bbox = kinematics::boundingBox(enlarged_linkage_region_pts);

				// calculate the circle point curve and center point curve
				for (int j = 0; j < synthesis.size(); j++) {
					if (!synthesis[j]) continue;
					synthesis[j]->calculateSolution(poses, enlarged_linkage_region_pts, linkage_avoidance_pts[0], num_samples, moving_bodies, initial_solutions);
					cnt += initial_solutions.size();
				}
			}

			time_t end = clock();
			std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for obtaining " << cnt << " candidates." << std::endl;

			if (cnt == 0) {
				mainWin->ui.statusBar->showMessage("No candidate was found.");
			}

			// use a default linkage if no solution is found
			if (cnt == 0) {
				for (int j = 0; j < synthesis.size(); j++) {
					if (!synthesis[j]) continue;
					kinematics::Kinematics kin;
					solutions.push_back(kinematics::Solution(0, { { 0, 0 }, { 0, 2 }, { 2, 0 }, { 2, 2 } }, 0, 0, poses));
					std::vector<glm::dvec2> connector_pts;
					kin = synthesis[j]->constructKinematics(poses, solutions.back().points, moving_bodies, true, fixed_bodies, connector_pts);
					kinematics.push_back(kin);
				}
			}

			start = clock();

			kinematics::Kinematics kin;
			selected_solution.cost = std::numeric_limits<double>::max();
			for (int j = 0; j < synthesis.size(); j++) {
				if (!synthesis[j]) continue;

				kinematics::Solution solution = synthesis[j]->findBestSolution(poses, initial_solutions, dist_map, dist_map_bbox, moving_bodies, num_particles, num_pf_iterations, record_pf);
				if (solution.cost < selected_solution.cost) {
					selected_solution = solution;
				}

				solutions.insert(solutions.end(), initial_solutions.begin(), initial_solutions.end());
			}

			std::vector<glm::dvec2> connector_pts;
			kin = synthesis[selected_solution.linkage_type]->constructKinematics(selected_solution.poses, selected_solution.points, moving_bodies, true, fixed_bodies, connector_pts);
			kinematics.push_back(kin);

			end = clock();
			std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for finding the best solution. " << std::endl;
		}

		// setup the kinematic system
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].initialize();
		}

		time_t end = clock();
		std::cout << "Total computation time was " << (double)(end - start) / CLOCKS_PER_SEC << " sec." << std::endl;

		update();
	}

	void Canvas::updateSolutions(int linkage_type) {
		mainWin->ui.statusBar->showMessage("Please wait for a moment...");

		// change the mode to kinematics
		setMode(MODE_KINEMATICS);
		mainWin->ui.actionKinematics->setChecked(true);

		this->linkage_type = linkage_type;

		kinematics.clear();
		synthesis.clear();
		synthesis.resize(1);

		synthesis[0] = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesisWattI(merged_fixed_bodies, { stddev_position, stddev_orientation }, avoid_branch_defect, min_transmission_angle, 1.0, weights));

		solutions.clear();
		{
			// calculate a distance mapt for the linkage region
			cv::Mat dist_map;
			kinematics::BBox dist_map_bbox;
			kinematics::LinkageSynthesis::createDistanceMapForLinkageRegion(linkage_region_pts[0], 5, dist_map_bbox, dist_map);

			// calculate the center of the valid regions
			kinematics::BBox bbox = kinematics::boundingBox(linkage_region_pts[0]);
			glm::dvec2 bbox_center = bbox.center();

			std::vector<glm::dvec2> enlarged_linkage_region_pts;

			int cnt = initial_solutions.size();

			if (cnt == 0) {
				mainWin->ui.statusBar->showMessage("No candidate was found.");
			}

			// use a default linkage if no solution is found
			if (cnt == 0) {
				for (int j = 0; j < synthesis.size(); j++) {
					if (!synthesis[j]) continue;
					kinematics::Kinematics kin;
					solutions.push_back(kinematics::Solution(0, { { 0, 0 }, { 0, 2 }, { 2, 0 }, { 2, 2 }, { 2, 4 }, { 4, 2 }, { 4, 4 } }, 0, 0, poses));
					std::vector<glm::dvec2> connector_pts;
					kin = synthesis[j]->constructKinematics(poses, solutions.back().points, moving_bodies, true, fixed_bodies, connector_pts);
					kinematics.push_back(kin);
				}
			}

			time_t start = clock();

			kinematics::Kinematics kin;
			selected_solution.cost = std::numeric_limits<double>::max();
			for (int j = 0; j < synthesis.size(); j++) {
				if (!synthesis[j]) continue;

				kinematics::Solution solution = synthesis[j]->findBestSolution(poses, initial_solutions, dist_map, dist_map_bbox, moving_bodies, num_particles, num_pf_iterations, record_pf);
				if (solution.cost < selected_solution.cost) {
					selected_solution = solution;
				}

				solutions.insert(solutions.end(), initial_solutions.begin(), initial_solutions.end());
			}

			std::vector<glm::dvec2> connector_pts;
			kin = synthesis[selected_solution.linkage_type]->constructKinematics(selected_solution.poses, selected_solution.points, moving_bodies, true, fixed_bodies, connector_pts);
			kinematics.push_back(kin);

			time_t end = clock();
			std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for finding the best solution. " << std::endl;
		}

		// setup the kinematic system
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].initialize();
		}

		update();
	}

	/**
	* Construct a kinematic diagram based on the selected solution.
	*/
	void Canvas::constructKinematics() {
		kinematics.clear();

		// construct kinamtics
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kin = synthesis[selected_solution.linkage_type]->constructKinematics(selected_solution.poses, selected_solution.points, moving_bodies, true, fixed_bodies, connector_pts);
		kinematics.push_back(kin);

		// setup the kinematic system
		for (int i = 0; i < kinematics.size(); i++) {
			kinematics[i].initialize();
		}
	}

	/**
	 * Find the closest solution.
	 * 
	 * @param solutions	solution set
	 * @param pt		mouse position
	 * @param joint_id	0 -- driving crank / 1 -- follower
	 */
	int Canvas::findSolution(const std::vector<kinematics::Solution>& solutions, const glm::dvec2& pt, int joint_id) {
		int ans = -1;
		double min_dist = std::numeric_limits<double>::max();

		for (int i = 0; i < solutions.size(); i++) {
			double dist = glm::length(solutions[i].points[joint_id] - pt);
			if (dist < min_dist) {
				min_dist = dist;
				ans = i;
			}
		}

		return ans;
	}

	void Canvas::animation_update() {
		for (int i = 0; i < kinematics.size(); i++) {
			try {
				kinematics[i].stepForward(collision_check, true, restrict_motion_range);
			}
			catch (char* ex) {
				kinematics[i].invertSpeed();
				//stop();
				std::cerr << "Animation is stopped by error:" << std::endl;
				std::cerr << ex << std::endl;
			}
		}

		update();

	}

	void Canvas::paintEvent(QPaintEvent *e) {
		QPainter painter(this);
		painter.fillRect(0, 0, width(), height(), QColor(255, 255, 255));

		// draw grid
		if (show_grid_lines) {
			painter.save();
			painter.setPen(QPen(QColor(224, 224, 224), 1));
			for (int i = -2000; i <= 2000; i++) {
				painter.drawLine(origin.x() + i * 5 * scale, -10000 * scale + origin.y(), origin.x() + i * 5 * scale, 10000 * scale + origin.y());
				painter.drawLine(-10000 * scale + origin.x(), origin.y() + i * 5 * scale, 10000 * scale + origin.x(), origin.y() + i * 5 * scale);
			}
			painter.restore();
		}

		if (mode != MODE_KINEMATICS) {
			// render unselected layers as background
			for (int i = 0; i < design.moving_bodies.size(); i++) {
				for (int j = 0; j < design.moving_bodies[i].poses.size(); j++) {
					if (j == design.layer_id) continue;
					design.moving_bodies[i].poses[j]->draw(painter, QColor(0, 255, 0, 60), origin, scale);
				}
			}

			// make the unselected layers faded
			painter.setPen(QColor(255, 255, 255, 160));
			painter.setBrush(QColor(255, 255, 255, 160));
			painter.drawRect(0, 0, width(), height());

			// render selected layer
			for (int i = 0; i < design.fixed_bodies.size(); i++) {
				design.fixed_bodies[i]->draw(painter, QColor(150, 255, 0, 60), origin, scale);
			}
			for (int i = 0; i < design.moving_bodies.size(); i++) {
				design.moving_bodies[i].poses[design.layer_id]->draw(painter, QColor(0, 255, 0, 60), origin, scale);
			}
			for (int i = 0; i < design.moving_bodies.size(); i++) {
				if (design.moving_bodies[i].linkage_region) {
					design.moving_bodies[i].linkage_region->draw(painter, QColor(0, 0, 255, 30), origin, scale);
				}
			}

			// render currently drawing shape
			if (mode == MODE_FIXED_RECTANGLE || mode == MODE_FIXED_CIRCLE || mode == MODE_FIXED_POLYGON || mode == MODE_MOVING_RECTANGLE || mode == MODE_MOVING_CIRCLE || mode == MODE_MOVING_POLYGON || mode == MODE_LINKAGE_REGION || mode == MODE_LINKAGE_AVOIDANCE) {
				if (current_shape) {
					current_shape->draw(painter, QColor(0, 0, 0, 0), origin, scale);
				}
			}
		}
		else {
			// draw solutions
			if (show_solutions && selectedJoint.first >= 0) {
				int linkage_id = selectedJoint.first;
				painter.setPen(QPen(QColor(128, 128, 255, 64), 1));
				painter.setBrush(QBrush(QColor(128, 128, 255, 64)));
				for (int i = 0; i < solutions.size(); i++) {
					if (selectedJoint.second < solutions[i].points.size()) {
						painter.drawEllipse(origin.x() + solutions[i].points[selectedJoint.second].x * scale, origin.y() - solutions[i].points[selectedJoint.second].y * scale, 3, 3);
					}
				}
			}

			// draw input shapes
			if (show_input_poses) {
				painter.setPen(QPen(QColor(0, 0, 0), 1, Qt::DashLine));
				painter.setBrush(QBrush(QColor(0, 0, 0, 0)));
				for (int i = 0; i < design.moving_bodies.size(); i++) {
					for (int j = 0; j < design.moving_bodies[i].poses.size(); j++) {
						QPolygonF pts;
						std::vector<glm::dvec2>& body = design.moving_bodies[i].poses[j]->getPoints();
						for (int k = 0; k < body.size(); k++) {
							pts.push_back(QPointF(origin.x() + body[k].x * scale, origin.y() - body[k].y * scale));
						}
						painter.drawPolygon(pts);
					}
				}
			}

			// draw 2D mechanism
			for (int i = 0; i < kinematics.size(); i++) {
				kinematics[i].draw(painter, origin, scale, show_linkage);
			}
		}

		// draw axes
		if (show_grid_lines) {
			painter.save();
			painter.setPen(QPen(QColor(128, 128, 128), 1));
			painter.drawLine(-10000 * scale + origin.x(), origin.y(), 10000 * scale + origin.x(), origin.y());
			painter.drawLine(origin.x(), -10000 * scale + origin.y(), origin.x(), 10000 * scale + origin.y());
			painter.restore();
		}
	}

	void Canvas::mousePressEvent(QMouseEvent* e) {
		ctrlPressed = e->modifiers() & Qt::ControlModifier;

		// This is necessary to get key event occured even after the user selects a menu.
		setFocus();

		if (e->buttons() & Qt::LeftButton) {
			if (mode == MODE_SELECT) {
				// hit test for rotation marker
				glm::dvec2 rotate_pivot;
				if (design.hitTestRotationMarker(screenToWorldCoordinates(e->x(), e->y()), scale, 10 / scale, selected_shape, rotate_pivot)) {
					// start rotating
					mode = MODE_ROTATION;
					operation = boost::shared_ptr<Operation>(new RotateOperation(screenToWorldCoordinates(e->x(), e->y()), rotate_pivot));
					update();
					return;
				}

				// hit test for resize marker
				glm::dvec2 resize_pivot;
				if (design.hitTestResizeMarker(screenToWorldCoordinates(e->x(), e->y()), 10 / scale, selected_shape, resize_pivot)) {
					// start resizing
					mode = MODE_RESIZE;
					operation = boost::shared_ptr<canvas::Operation>(new canvas::ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), resize_pivot));
					update();
					return;
				}

				// hit test for the shape
				if (design.hitTest(screenToWorldCoordinates(e->x(), e->y()), ctrlPressed, selected_shape)) {
					// start moving
					mode = MODE_MOVE;
					operation = boost::shared_ptr<canvas::Operation>(new canvas::MoveOperation(screenToWorldCoordinates(e->x(), e->y())));
					update();
					return;
				}

				unselectAll();
			}
			else if (mode == MODE_FIXED_RECTANGLE || mode == MODE_MOVING_RECTANGLE) {
				if (!current_shape) {
					// start drawing a rectangle
					unselectAll();
					current_shape = boost::shared_ptr<Shape>(new Rectangle(screenToWorldCoordinates(e->x(), e->y())));
					current_shape->startDrawing();
					setMouseTracking(true);
				}
			}
			else if (mode == MODE_FIXED_CIRCLE || mode == MODE_MOVING_CIRCLE) {
				if (!current_shape) {
					// start drawing a circle
					unselectAll();
					current_shape = boost::shared_ptr<Shape>(new Circle(screenToWorldCoordinates(e->x(), e->y())));
					current_shape->startDrawing();
					setMouseTracking(true);
				}
			}
			else if (mode == MODE_FIXED_POLYGON || mode == MODE_MOVING_POLYGON) {
				if (current_shape) {
					current_shape->addPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())));
				}
				else {
					// start drawing a polygon
					unselectAll();
					current_shape = boost::shared_ptr<Shape>(new Polygon(screenToWorldCoordinates(e->x(), e->y())));
					current_shape->startDrawing();
					setMouseTracking(true);
				}
			}
			else if (mode == MODE_LINKAGE_REGION) {
				if (current_shape) {
					current_shape->addPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())));
				}
				else {
					// start drawing a polygon
					unselectAll();
					current_shape = boost::shared_ptr<Shape>(new Polygon(screenToWorldCoordinates(e->x(), e->y())));
					current_shape->startDrawing();
					setMouseTracking(true);
				}
			}
			else if (mode == MODE_KINEMATICS) {
				// convert the mouse position to the world coordinate system
				glm::dvec2 pt((e->x() - origin.x()) / scale, -(e->y() - origin.y()) / scale);

				// select a joint to move
				selectedJoint = std::make_pair(-1, -1);
				double min_dist = 6;
				for (int i = 0; i < kinematics.size(); i++) {
					for (int j = 0; j < kinematics[i].diagram.joints.size(); j++) {
						if (!ctrlPressed && j >= 2) continue;
						double dist = glm::length(kinematics[i].diagram.joints[j]->pos - pt);
						if (dist < min_dist) {
							min_dist = dist;
							selectedJoint = std::make_pair(i, j);
						}
					}
				}
			}
		}

		prev_mouse_pt = e->pos();
	}

	void Canvas::mouseMoveEvent(QMouseEvent* e) {
		if (e->buttons() & Qt::RightButton) {
			// move the camera
			if (e->buttons() & Qt::RightButton) {
				// translate the Origin
				origin += e->pos() - prev_mouse_pt;
				update();
			}
		}
		else {
			if (mode == MODE_MOVE) {
				boost::shared_ptr<canvas::MoveOperation> op = boost::static_pointer_cast<canvas::MoveOperation>(operation);
				glm::dvec2 dir = screenToWorldCoordinates(e->x(), e->y()) - op->pivot;
				design.move(dir);

				op->pivot = screenToWorldCoordinates(e->x(), e->y());
				update();
			}
			else if (mode == MODE_ROTATION) {
				boost::shared_ptr<canvas::RotateOperation> op = boost::static_pointer_cast<canvas::RotateOperation>(operation);
				glm::dvec2 dir1 = op->pivot - op->rotation_center;
				glm::dvec2 dir2 = screenToWorldCoordinates(e->x(), e->y()) - op->rotation_center;
				double theta = atan2(dir2.y, dir2.x) - atan2(dir1.y, dir1.x);
				design.rotate(theta);

				op->pivot = screenToWorldCoordinates(e->x(), e->y());
				update();
			}
			else if (mode == MODE_RESIZE) {
				boost::shared_ptr<canvas::ResizeOperation> op = boost::static_pointer_cast<canvas::ResizeOperation>(operation);
				glm::dvec2 resize_center = selected_shape->localCoordinate(op->resize_center);
				glm::dvec2 dir1 = selected_shape->localCoordinate(op->pivot) - resize_center;
				glm::dvec2 dir2 = selected_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())) - resize_center;
				glm::dvec2 resize_scale(dir2.x / dir1.x, dir2.y / dir1.y);
				design.resize(resize_scale, resize_center);

				op->pivot = screenToWorldCoordinates(e->x(), e->y());
				update();
			}
			else if (mode == MODE_FIXED_RECTANGLE || mode == MODE_FIXED_CIRCLE || mode == MODE_FIXED_POLYGON || mode == MODE_MOVING_RECTANGLE || mode == MODE_MOVING_CIRCLE || mode == MODE_MOVING_POLYGON || mode == MODE_LINKAGE_REGION) {
				if (current_shape) {
					current_shape->updateByNewPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())), shiftPressed);
					update();
				}
			}
			else if (mode == MODE_KINEMATICS) {
				if (selectedJoint.first >= 0) {
					int linkage_id = selectedJoint.first;
					int joint_id = selectedJoint.second;

					// select a solution
					glm::dvec2 pt = screenToWorldCoordinates(e->x(), e->y());
					int selectedSolution = findSolution(solutions, pt, joint_id);
					if (selectedSolution >= 0) {
						selected_solution = solutions[selectedSolution];

						std::cout << "Selected linkage:" << std::endl;
						for (int i = 0; i < selected_solution.points.size(); i++) {
							std::cout << "(" << selected_solution.points[i].x << "," << selected_solution.points[i].y << ") ";
						}
						std::cout << std::endl;
					}

					// update the geometry
					constructKinematics();
					update();
				}
			}
		}
		
		prev_mouse_pt = e->pos();
	}

	void Canvas::mouseReleaseEvent(QMouseEvent* e) {
		if (e->button() == Qt::RightButton) {
		}
		else if (mode == MODE_MOVE || mode == MODE_ROTATION || mode == MODE_RESIZE) {
			history.push(design);
			mode = MODE_SELECT;
		}
		else if (mode == MODE_KINEMATICS) {
			if (selectedJoint.first >= 0) {
				constructKinematics();
			}
		}

		update();
	}

	void Canvas::mouseDoubleClickEvent(QMouseEvent* e) {
		if (mode == MODE_FIXED_RECTANGLE || mode == MODE_FIXED_CIRCLE || mode == MODE_FIXED_POLYGON) {
			// The shape is created.
			current_shape->completeDrawing();
			design.fixed_bodies.push_back(current_shape->clone());
			design.fixed_bodies.back()->select();
		}
		else if (mode == MODE_MOVING_RECTANGLE || mode == MODE_MOVING_CIRCLE || mode == MODE_MOVING_POLYGON) {
			// The shape is created.
			current_shape->completeDrawing();
			design.addMovingBody(current_shape);
		}
		else if (mode == MODE_LINKAGE_REGION) {
			// The shape is created.
			current_shape->completeDrawing();
			for (int i = 0; i < design.moving_bodies.size(); i++) {
				if (!design.moving_bodies[i].linkage_region) {
					design.moving_bodies[i].linkage_region = current_shape->clone();
					design.moving_bodies[i].linkage_region->select();
					break;
				}
			}
		}

		if (mode == MODE_FIXED_RECTANGLE || mode == MODE_FIXED_CIRCLE || mode == MODE_FIXED_POLYGON || mode == MODE_MOVING_RECTANGLE || mode == MODE_MOVING_CIRCLE || mode == MODE_MOVING_POLYGON || mode == MODE_LINKAGE_REGION) {
			mode = MODE_SELECT;
			history.push(design);
			current_shape.reset();
			operation.reset();
			mainWin->ui.actionSelect->setChecked(true);
		}

		setMouseTracking(false);

		update();
	}

	void Canvas::wheelEvent(QWheelEvent* e) {
		double new_scale = scale + e->delta() * 0.01;
		new_scale = std::min(std::max(0.1, new_scale), 1000.0);

		// adjust the origin in order to keep the screen center
		glm::dvec2 p = screenToWorldCoordinates(width() * 0.5, height() * 0.5);
		origin.setX(width() * 0.5 - p.x * new_scale);
		origin.setY(height() * 0.5 + p.y * new_scale);
		
		scale = new_scale;

		update();
	}

	void Canvas::resizeEvent(QResizeEvent *e) {
		// adjust the origin in order to keep the screen center
		glm::dvec2 p = screenToWorldCoordinates(e->oldSize().width() * 0.5, e->oldSize().height() * 0.5);
		origin.setX(e->size().width() * 0.5 - p.x * scale);
		origin.setY(e->size().height() * 0.5 + p.y * scale);
	}

	void Canvas::keyPressEvent(QKeyEvent* e) {
		ctrlPressed = false;
		shiftPressed = false;

		if (e->modifiers() & Qt::ControlModifier) {
			ctrlPressed = true;
		}
		if (e->modifiers() & Qt::ShiftModifier) {
			shiftPressed = true;
		}

		switch (e->key()) {
		case Qt::Key_Space:
			// start/stop the animation
			if (animation_timer == NULL) {
				run();
			}
			else {
				stop();
			}
			break;
		case Qt::Key_Up:
			for (int i = 0; i < kinematics.size(); i++) {
				kinematics[i].simulation_speed *= 2;
			}
			break;
		case Qt::Key_Down:
			for (int i = 0; i < kinematics.size(); i++) {
				kinematics[i].simulation_speed *= 0.5;
			}
			break;
		default:
			break;
		}

		update();
	}

	void Canvas::keyReleaseEvent(QKeyEvent* e) {
		switch (e->key()) {
		case Qt::Key_Control:
			ctrlPressed = false;
			break;
		case Qt::Key_Shift:
			shiftPressed = false;
			break;
		default:
			break;
		}
	}

}
