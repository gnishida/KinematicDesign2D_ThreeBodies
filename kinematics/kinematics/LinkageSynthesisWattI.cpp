#include "LinkageSynthesisWattI.h"
#include <opencv2/opencv.hpp>
#include "KinematicUtils.h"
#include "Kinematics.h"
#include "PinJoint.h"
#include "SliderHinge.h"
#include "BoundingBox.h"
#include "LeastSquareSolver.h"

namespace kinematics {

	LinkageSynthesisWattI::LinkageSynthesisWattI(const std::vector<Object25D>& fixed_bodies, const std::vector<std::pair<double, double>>& sigmas, bool avoid_branch_defect, double min_transmission_angle, double min_link_length, const std::vector<double>& weights) {
		this->fixed_bodies = fixed_bodies;
		this->sigmas = sigmas;
		this->avoid_branch_defect = avoid_branch_defect;
		this->min_transmission_angle = min_transmission_angle;
		this->min_link_length = min_link_length;
		this->weights = weights;
	}

	/**
	* Calculate solutions of Watt I.
	*
	* @param poses			three poses
	* @param solutions1	the output solutions for the world coordinates of the driving crank at the first pose, each of which contains a pair of the center point and the circle point
	* @param solutions2	the output solutions for the world coordinates of the follower at the first pose, each of which contains a pair of the center point and the circle point
	*/
	void LinkageSynthesisWattI::calculateSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const std::vector<Object25D>& moving_bodies, std::vector<Solution>& solutions) {
		solutions.clear();

		srand(0);

		int cnt = 0;

		// calculate the bounding boxe of the valid regions
		BBox bbox = boundingBox(linkage_region_pts);

		for (int iter = 0; iter < num_samples && cnt < num_samples; iter++) {
			printf("\rsampling %d/%d", cnt, iter + 1);

			// perturbe the poses a little
			double position_error = 0.0;
			double orientation_error = 0.0;
			std::vector<std::vector<glm::dmat3x3>> perturbed_poses = perturbPoses(poses, sigmas, position_error, orientation_error);

			// sample joints within the linkage region
			std::vector<glm::dvec2> points(10);
			for (int i = 0; i < points.size(); i++) {
				while (true) {
					points[i] = glm::dvec2(genRand(bbox.minPt.x, bbox.maxPt.x), genRand(bbox.minPt.y, bbox.maxPt.y));
					if (withinPolygon(linkage_region_pts, points[i])) break;
				}
			}
			
			// DEBUG
			if (iter == 0) {
				points[0] = glm::dvec2(-3.76487, 25.9907);
				points[1] = glm::dvec2(-4.06804, 17.4689);
				points[2] = glm::dvec2(-21.797, 38.1363);
				points[3] = glm::dvec2(-0.900912, 22.4179);
				points[4] = glm::dvec2(-20.1314, 30.8356);
				points[5] = glm::dvec2(-22.5043, 33.0577);
				points[6] = glm::dvec2(-26.1766, 29.1351);
				points[7] = glm::dvec2(-42.8608, 18.9469);
				points[8] = glm::dvec2(-47.912, 19.7987);
				points[9] = glm::dvec2(-39.0381, 24.691);
			}

			if (iter > 0 && !optimizeCandidate(perturbed_poses, linkage_region_pts, bbox, points)) continue;

			// check hard constraints
			if (iter > 0 && !checkHardConstraints(points, perturbed_poses, linkage_region_pts, linkage_avoidance_pts, moving_bodies)) continue;
			
			solutions.push_back(Solution(0, points, 0, 0, perturbed_poses));
			cnt++;
		}

		printf("\n");
	}

	/**
	* Optimize the linkage parameters based on the rigidity constraints.
	* If it fails to optimize, return false.
	*/
	bool LinkageSynthesisWattI::optimizeCandidate(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points) {
		if (!optimizeLink(poses, linkage_region_pts, bbox, points)) return false;
		if (check(poses, points) > 1.0) return false;

		// Check if all the joints are within the linkage region
		for (int i = 0; i < points.size(); i++) {
			if (!withinPolygon(linkage_region_pts, points[i])) return false;
		}

		return true;
	}

	/**
	 * @param linkage_region_pts_local	linkage region in the local coordinate system of moving objects (i = 0, 1)
	 * @param bbox_local				bounding box in the local coordinate system of moving objects (i = 0, 1)
	 */
	bool LinkageSynthesisWattI::optimizeLink(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points) {
		// setup the initial parameters for optimization
		column_vector starting_point(points.size() * 2);
		for (int i = 0; i < points.size(); i++) {
			starting_point(i * 2, 0) = points[i].x;
			starting_point(i * 2 + 1, 0) = points[i].y;
		}

		try {
			find_min(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), SolverForWattI(poses), SolverDerivForWattI(poses), starting_point, -1);
			//find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), SolverForWattI(poses), starting_point, -1);
			
			for (int i = 0; i < points.size(); i++) {
				points[i] = glm::dvec2(starting_point(i * 2, 0), starting_point(i * 2 + 1, 0));
			}
		}
		catch (std::exception& e) {
			return false;
		}

		return true;
	}

	Solution LinkageSynthesisWattI::findBestSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& moving_bodies, int num_particles, int num_iterations, bool record_file) {
		// select the best solution based on the trajectory
		if (solutions.size() > 0) {
			particleFilter(solutions, linkage_region_pts, dist_map, dist_map_bbox, linkage_avoidance_pts, moving_bodies, num_particles, num_iterations, record_file);
			return solutions[0];
		}
		else {
			return Solution(0, { { 0, 0 }, { 2, 0 }, { 0, 2 }, { 2, 2 }, { 1, 3 }, { 3, 3 }, { 3, 5 }, { 5, 7 }, { 6, 8 }, { 8, 10 } }, 0, 0, poses);
		}
	}

	double LinkageSynthesisWattI::calculateCost(Solution& solution, const std::vector<Object25D>& moving_bodies, const cv::Mat& dist_map, const BBox& dist_map_bbox) {
		double dist = 0;
		for (int i = 0; i < solution.points.size(); i++) {
			int r = solution.points[i].y - dist_map_bbox.minPt.y;
			int c = solution.points[i].x - dist_map_bbox.minPt.x;
			if (r >= 0 && r < dist_map.rows && c >= 0 && c < dist_map.cols) dist += dist_map.at<double>(r, c);
			else dist += dist_map.rows + dist_map.cols;
		}
		double tortuosity = tortuosityOfTrajectory(solution.poses, solution.points, moving_bodies);
		std::vector<glm::dvec2> connected_pts;
		Kinematics kin = constructKinematics(solution.poses, solution.points, moving_bodies, true, fixed_bodies, connected_pts);
		//double size = glm::length(solution.points[0] - solution.points[2]) + glm::length(solution.points[1] - solution.points[3]) + glm::length(solution.points[0] - connected_pts[0]) + glm::length(solution.points[1] - connected_pts[1]) + glm::length(solution.points[2] - connected_pts[2]) + glm::length(solution.points[3] - connected_pts[3]);
		double size = glm::length(solution.points[0] - solution.points[2]) + glm::length(solution.points[1] - solution.points[3]) + std::max(std::max(glm::length(solution.points[2] - solution.points[3]), glm::length(solution.points[2] - solution.points[5])), glm::length(solution.points[3] - solution.points[5])) + glm::length(solution.points[1] - solution.points[4]) + glm::length(solution.points[3] - solution.points[4]) + glm::length(solution.points[5] - solution.points[6]);

		return solution.position_error * weights[0] + solution.orientation_error * weights[0] * 10 + dist * weights[1] + (tortuosity - 1) * weights[2] + size * weights[3];
	}

	int LinkageSynthesisWattI::getType(const std::vector<glm::dvec2>& points) {
		double g = glm::length(points[0] - points[1]);
		double a = glm::length(points[0] - points[2]);
		double b = glm::length(points[1] - points[3]);
		double h = glm::length(points[2] - points[3]);

		double T1 = g + h - a - b;
		double T2 = b + g - a - h;
		double T3 = b + h - a - g;

		if (T1 < 0 && T2 < 0 && T3 >= 0) {
			return 0;
		}
		else if (T1 >= 0 && T2 >= 0 && T3 >= 0) {
			return 1;
		}
		else if (T1 >= 0 && T2 < 0 && T3 < 0) {
			return 2;
		}
		else if (T1 < 0 && T2 >= 0 && T3 < 0) {
			return 3;
		}
		else if (T1 < 0 && T2 < 0 && T3 < 0) {
			return 4;
		}
		else if (T1 < 0 && T2 >= 0 && T3 >= 0) {
			return 5;
		}
		else if (T1 >= 0 && T2 < 0 && T3 >= 0) {
			return 6;
		}
		else if (T1 >= 0 && T2 >= 0 && T3 < 0) {
			return 7;
		}
		else {
			return -1;
		}
	}

	/**
	* Check if the linkage has a rotatable crank defect.
	* If the crank is not fully rotatable, true is returned.
	*/
	bool LinkageSynthesisWattI::checkRotatableCrankDefect(const std::vector<glm::dvec2>& points) {
		int linkage_type = getType(points);
		int linkage_type2 = getType({ points[3], points[4], points[5], points[6] });

		if ((linkage_type == 0 || linkage_type == 1) && (linkage_type2 == 0 || linkage_type2 == 1)) {
			return false;
		}
		else {
			return true;
		}
	}

	std::pair<double, double> LinkageSynthesisWattI::checkRange(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		if (poses[0].size() == 2) {
			glm::dvec2 dir1 = points[2] - points[0];
			glm::dvec2 dir2 = glm::dvec2(poses[0].back() * glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1)) - points[0];

			double angle1 = std::atan2(dir1.y, dir1.x);
			double angle2 = std::atan2(dir2.y, dir2.x);

			// try clock-wise order
			double a1 = angle1;
			double a2 = angle2;
			if (a2 > a1) {
				a2 -= M_PI * 2;
			}
			if (a1 - a2 < M_PI) {
				return{ std::min(a1, a2), std::max(a1, a2) };
			}

			// try counter-clock-wise order
			if (angle2 < angle1) {
				angle2 += M_PI * 2;
			}
			return{ std::min(angle1, angle2), std::max(angle1, angle2) };
		}
		else {
			glm::dvec2 dir1 = points[2] - points[0];
			glm::dvec2 dir2 = glm::dvec2(poses[0][1] * glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1)) - points[0];
			glm::dvec2 dir3 = glm::dvec2(poses[0].back() * glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1)) - points[0];

			double angle1 = std::atan2(dir1.y, dir1.x);
			double angle2 = std::atan2(dir2.y, dir2.x);
			double angle3 = std::atan2(dir3.y, dir3.x);

			// try clock-wise order
			double a1 = angle1;
			double a2 = angle2;
			double a3 = angle3;
			if (a2 > a1) {
				a2 -= M_PI * 2;
			}
			while (a3 > a2) {
				a3 -= M_PI * 2;
			}
			if (a1 - a3 < M_PI * 2) {
				return{ std::min(a1, a3), std::max(a1, a3) };
			}

			// try counter-clock-wise order
			if (angle2 < angle1) {
				angle2 += M_PI * 2;
			}
			if (angle3 < angle2) {
				angle3 += M_PI * 2;
			}
			if (angle3 - angle1 < M_PI * 2) {
				return{ std::min(angle1, angle3), std::max(angle1, angle3) };
			}
		}
		
		return{ 0, 0 };
	}

	bool LinkageSynthesisWattI::checkOrderDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		glm::dvec2 inv_W = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));

		int linkage_type = getType(points);
		std::pair<double, double> range = checkRange(poses, points);

		double total_cw = 0;
		double total_ccw = 0;
		double prev = 0;
		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving crank in the world coordinate system
			glm::dvec2 X = glm::dvec2(poses[0][i] * glm::dvec3(inv_W, 1));
			//std::cout << X.x << "," << X.y << std::endl;

			// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
			glm::dvec2 dir = X - points[0];

			// calculate its angle
			double theta = atan2(dir.y, dir.x);

			if (theta >= prev) {
				if (linkage_type == 0 || linkage_type == 2) {
					total_cw += M_PI * 2 - theta + prev;
					total_ccw += theta - prev;
				}
				else if (linkage_type == 2 || linkage_type == 3 || linkage_type == 4 || linkage_type == 7) {
					total_cw = M_PI * 999; // out of range
					total_ccw += theta - prev;
				}
				else if (linkage_type == 5 || linkage_type == 6) {
					if (theta < range.first) {
						theta += M_PI * 2;
					}
					total_cw = M_PI * 999; // out of range
					total_ccw += theta - prev;
				}
			}
			else {
				if (linkage_type == 0 || linkage_type == 2) {
					total_cw += prev - theta;
					total_ccw += M_PI * 2 - prev + theta;
				}
				else if (linkage_type == 2 || linkage_type == 3 || linkage_type == 4 || linkage_type == 7) {
					total_cw += prev - theta;
					total_ccw = M_PI * 999;	// out of range
				}
				else if (linkage_type == 5 || linkage_type == 6) {
					if (theta < range.first) {
						theta += M_PI * 2;
					}
					total_cw += prev - theta;
					total_ccw = M_PI * 999;	// out of range
				}
			}

			prev = theta;
		}

		if (total_cw > M_PI * 2 && total_ccw > M_PI * 2) return true;
		else return false;
	}

	/**
	 * Check if all the poses are in the same branch.
	 * Drag-link and crank-rocker always do not have a branch defect.
	 * For other types of linkage, the change in the sign of the angle between the coupler and the follower indicates the change of the branch.
	 * If there is an branch defect, true is returned. Otherwise, false is returned.
	 *
	 * @param poses	pose matrices
	 * @param p0		the world coordinates of the fixed point of the driving crank at the first pose
	 * @param p1		the world coordinates of the fixed point of the follower at the first pose
	 * @param p2		the world coordinates of the moving point of the driving crank at the first pose
	 * @param p3		the world coordinates of the moving point of the follower at the first pose
	 * @return		true if the branch defect is detected, false otherwise
	 */
	bool LinkageSynthesisWattI::checkBranchDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		// drag-link and crank-rocker always do not have a branch defect
		if (type == 0 || type == 1) return false;

		int orig_sign = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 p2 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));
		glm::dvec2 p3 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[3], 1));

		for (int i = 0; i < poses[0].size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P2 = glm::dvec2(poses[0][i] * glm::dvec3(p2, 1));
			glm::dvec2 P3 = glm::dvec2(poses[0][i] * glm::dvec3(p3, 1));

			// calculate its sign
			if (i == 0) {
				orig_sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
			}
			else {
				int sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
				if (sign != orig_sign) {
					return true;
				}
			}
		}

		return false;
	}

	bool LinkageSynthesisWattI::checkCircuitDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		int type = getType(points);

		int orig_sign = 1;
		int orig_sign2 = 1;
		int orig_sign3 = 1;	// sign for the link 1-3-4
		int orig_sign4 = 1;
		int orig_sign5 = 1;

		// calculate the local coordinates of the circle points
		glm::dvec2 p2 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));
		glm::dvec2 p3 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[3], 1));
		glm::dvec2 p4 = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(points[4], 1));
		glm::dvec2 p5 = glm::dvec2(glm::inverse(poses[0][0]) * glm::dvec3(points[5], 1));
		glm::dvec2 p6 = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(points[6], 1));
		glm::dvec2 p7 = glm::dvec2(glm::inverse(poses[1][0]) * glm::dvec3(points[7], 1));
		glm::dvec2 p8 = glm::dvec2(glm::inverse(poses[2][0]) * glm::dvec3(points[8], 1));
		glm::dvec2 p9 = glm::dvec2(glm::inverse(poses[2][0]) * glm::dvec3(points[9], 1));

		for (int i = 0; i < poses[0].size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 P2 = glm::dvec2(poses[0][i] * glm::dvec3(p2, 1));
			glm::dvec2 P3 = glm::dvec2(poses[0][i] * glm::dvec3(p3, 1));
			glm::dvec2 P4 = glm::dvec2(poses[1][i] * glm::dvec3(p4, 1));
			glm::dvec2 P5 = glm::dvec2(poses[0][i] * glm::dvec3(p5, 1));
			glm::dvec2 P6 = glm::dvec2(poses[1][i] * glm::dvec3(p6, 1));
			glm::dvec2 P7 = glm::dvec2(poses[1][i] * glm::dvec3(p7, 1));
			glm::dvec2 P8 = glm::dvec2(poses[2][i] * glm::dvec3(p8, 1));
			glm::dvec2 P9 = glm::dvec2(poses[2][i] * glm::dvec3(p9, 1));

			// calculate its sign
			if (i == 0) {
				if (type == 0) {
					orig_sign = crossProduct(P2 - points[0], P3 - P2) >= 0 ? 1 : -1;
				}
				else if (type == 1) {
					orig_sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
				}
				else if (type == 2) {
					orig_sign = crossProduct(points[0] - points[1], P2 - points[0]) >= 0 ? 1 : -1;
				}
				else if (type == 3) {
					orig_sign = crossProduct(points[1] - P3, points[0] - points[1]) >= 0 ? 1 : -1;
				}

				orig_sign2 = crossProduct(P6 - P5, P4 - P6) >= 0 ? 1 : -1;
				orig_sign3 = crossProduct(P4 - points[1], P3 - P4) >= 0 ? 1 : -1;
				orig_sign4 = crossProduct(P9 - P8, P7 - P9) >= 0 ? 1 : -1;
				orig_sign5 = crossProduct(P8 - P5, P6 - P8) >= 0 ? 1 : -1;
			}
			else {
				int sign, sign2, sign3, sign4, sign5;
				if (type == 0) {
					sign = crossProduct(P2 - points[0], P3 - P2) >= 0 ? 1 : -1;
				}
				else if (type == 1) {
					sign = crossProduct(P3 - P2, points[1] - P3) >= 0 ? 1 : -1;
				}
				else if (type == 2) {
					sign = crossProduct(points[0] - points[1], P2 - points[0]) >= 0 ? 1 : -1;
				}
				else if (type == 3) {
					sign = crossProduct(points[1] - P3, points[0] - points[1]) >= 0 ? 1 : -1;
				}
				else {
					sign = orig_sign;
				}

				sign2 = crossProduct(P6 - P5, P4 - P6) >= 0 ? 1 : -1;
				sign3 = crossProduct(P4 - points[1], P3 - P4) >= 0 ? 1 : -1;
				sign4 = crossProduct(P9 - P8, P7 - P9) >= 0 ? 1 : -1;
				sign5 = crossProduct(P8 - P5, P6 - P8) >= 0 ? 1 : -1;

				if (sign != orig_sign || sign2 != orig_sign2 || sign3 != orig_sign3 || sign4 != orig_sign4 || sign5 != orig_sign5) return true;
			}
		}

		return false;
	}

	/**
	* Construct a linkage.
	*/
	Kinematics LinkageSynthesisWattI::constructKinematics(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& moving_bodies, bool connect_joints, const std::vector<Object25D>& fixed_bodies, std::vector<glm::dvec2>& connected_pts) {
		Kinematics kin;
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, points[0])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, points[1])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, points[2])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(3, false, points[3])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(4, false, points[4])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(5, false, points[5])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(6, false, points[6])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(7, false, points[7])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(8, false, points[8])));
		kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(9, false, points[9])));
		kin.diagram.addLink(true, kin.diagram.joints[0], kin.diagram.joints[2], true);
		kin.diagram.addLink(false, { kin.diagram.joints[1], kin.diagram.joints[3], kin.diagram.joints[4] }, true);
		kin.diagram.addLink(false, { kin.diagram.joints[2], kin.diagram.joints[3], kin.diagram.joints[5] }, true);
		kin.diagram.addLink(false, { kin.diagram.joints[4], kin.diagram.joints[6], kin.diagram.joints[7] }, true);
		kin.diagram.addLink(false, { kin.diagram.joints[5], kin.diagram.joints[6], kin.diagram.joints[8] }, true);
		kin.diagram.addLink(false, kin.diagram.joints[7], kin.diagram.joints[9], true);
		kin.diagram.addLink(false, kin.diagram.joints[8], kin.diagram.joints[9], false);

		std::vector<Object25D> copied_fixed_bodies = fixed_bodies;

		// update the geometry
		updateMovingBodies(kin, moving_bodies);

		if (connect_joints) {
			kin.diagram.connectJointsToBodies(copied_fixed_bodies, connected_pts);
		}

		// add the fixed rigid bodies
		for (int i = 0; i < copied_fixed_bodies.size(); i++) {
			kin.diagram.addBody(kin.diagram.joints[0], kin.diagram.joints[1], copied_fixed_bodies[i]);
		}

		// calculte the range of motion
		std::pair<double, double> angle_range = checkRange(poses, points);
		kin.min_angle = angle_range.first;
		kin.max_angle = angle_range.second;

		return kin;
	}

	/**
	* Construct a linkage.
	*/
	void LinkageSynthesisWattI::updateMovingBodies(Kinematics& kin, const std::vector<Object25D>& moving_bodies) {
		kin.diagram.bodies.clear();
		kin.diagram.addBody(kin.diagram.joints[2], kin.diagram.joints[5], moving_bodies[0]);
		kin.diagram.addBody(kin.diagram.joints[4], kin.diagram.joints[7], moving_bodies[1]);
		kin.diagram.addBody(kin.diagram.joints[8], kin.diagram.joints[9], moving_bodies[2]);
	}

	bool LinkageSynthesisWattI::checkHardConstraints(std::vector<glm::dvec2>& points, const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& moving_bodies) {
		if (glm::length(points[0] - points[1]) < min_link_length) return false;
		if (glm::length(points[2] - points[3]) < min_link_length) return false;

		//if (checkFolding(points)) continue;
		if (avoid_branch_defect && checkBranchDefect(poses, points)) return false;
		if (checkCircuitDefect(poses, points)) return false;
		//if (checkOrderDefect(poses, points)) return false;

		if (!withinLinkageRegion(points, poses, fixed_bodies, moving_bodies, linkage_region_pts)) return false;

		// collision check
		if (checkCollision(poses, points, fixed_bodies, moving_bodies)) return false;

		return true;
	}

	bool LinkageSynthesisWattI::checkCollision(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& fixed_bodies, const std::vector<Object25D>& moving_bodies) {
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kinematics = constructKinematics(poses, points, moving_bodies, false, fixed_bodies, connector_pts);
		kinematics.diagram.initialize();

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 w(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));
		for (int i = 0; i < 2; i++) {
			glm::dvec2 W = glm::dvec2(poses[0][i] * glm::dvec3(w, 1));
			angles[i] = atan2(W.y - points[0].y, W.x - points[0].x);
		}
		{
			glm::dvec2 W = glm::dvec2(poses[0].back() * glm::dvec3(w, 1));
			angles[2] = atan2(W.y - points[0].y, W.x - points[0].x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses[0].size() >= 3 && angles[1] >= angles[2] || poses[0].size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses[0].size() >= 3 && angles[1] < angles[2] || poses[0].size() == 2 && angles[0] - angles[1] > M_PI)) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(angles.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(2, false);
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return true;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - points[0].y, kinematics.diagram.joints[2]->pos.x - points[0].x);

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}

			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				kinematics.clear();
				return false;
			}
		}

		kinematics.clear();
		return false;
	}

	bool LinkageSynthesisWattI::withinLinkageRegion(std::vector<glm::dvec2>& points, const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<Object25D>& fixed_bodies, const std::vector<Object25D>& moving_bodies, const std::vector<glm::dvec2>& linkage_region_pts) {
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kinematics = constructKinematics(poses, points, moving_bodies, true, fixed_bodies, connector_pts);
		kinematics.diagram.initialize();

		if (!withinPolygon(linkage_region_pts, (points[0] + points[2]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[1] + points[3]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[1] + points[4]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[2] + points[3]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[2] + points[5]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[3] + points[4]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[3] + points[5]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[4] + points[6]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[4] + points[7]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[5] + points[6]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[5] + points[8]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[6] + points[7]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[6] + points[8]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[7] + points[9]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[8] + points[9]) * 0.5)) return false;

		if (!withinPolygon(linkage_region_pts, (points[0] + connector_pts[0]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[1] + connector_pts[1]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[2] + connector_pts[2]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[5] + connector_pts[3]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[4] + connector_pts[4]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[7] + connector_pts[5]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[8] + connector_pts[6]) * 0.5)) return false;
		if (!withinPolygon(linkage_region_pts, (points[9] + connector_pts[7]) * 0.5)) return false;

		return true;
	}

	double LinkageSynthesisWattI::tortuosityOfTrajectory(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& moving_bodies) {
		// calculate the local coordinates of the body points
		std::vector<std::vector<glm::dvec2>> body_pts_local(moving_bodies.size());
		for (int i = 0; i < moving_bodies.size(); i++) {
			body_pts_local[i].resize(moving_bodies[i].polygons[0].points.size());
			glm::dmat3x3 inv_pose0 = glm::inverse(poses[i][0]);
			for (int j = 0; j < moving_bodies[i].polygons[0].points.size(); j++) {
				body_pts_local[i][j] = glm::dvec2(inv_pose0 * glm::dvec3(moving_bodies[i].polygons[0].points[j], 1));
			}
		}

		// calculate the length of the motion using straight lines between poses
		double length_of_straight = 0.0;
		std::vector<std::vector<glm::dvec2>> prev_body_pts(moving_bodies.size());
		for (int i = 0; i < moving_bodies.size(); i++) {
			prev_body_pts[i] = moving_bodies[i].polygons[0].points;

			for (int j = 1; j < poses[i].size(); j++) {
				std::vector<glm::dvec2> next_body_pts(moving_bodies[i].polygons[0].points.size());
				for (int k = 0; k < moving_bodies[i].polygons[0].points.size(); k++) {
					next_body_pts[k] = glm::dvec2(poses[i][j] * glm::dvec3(body_pts_local[i][k], 1));
					length_of_straight += glm::length(next_body_pts[k] - prev_body_pts[i][k]);
				}
				prev_body_pts[i] = next_body_pts;
			}
		}


		// create a kinematics
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kinematics = constructKinematics(poses, points, moving_bodies, false, {}, connector_pts);
		kinematics.diagram.initialize();

		// initialize the trajectory of the moving body
		for (int i = 0; i < moving_bodies.size(); i++) {
			prev_body_pts[i] = moving_bodies[i].polygons[0].points;
		}
		double length_of_trajectory = 0.0;

		// calculate the rotational angle of the driving crank for 1st, 2nd, and last poses
		// i.e., angles[0] = first pose, angles[1] = second pose, angles[2] = last pose
		std::vector<double> angles(3);
		glm::dvec2 p2(glm::inverse(poses[0][0]) * glm::dvec3(points[2], 1));
		glm::dvec2 p3(glm::inverse(poses[0][0]) * glm::dvec3(points[3], 1));

		for (int i = 0; i < 2; i++) {
			glm::dvec2 P2(poses[0][i] * glm::dvec3(p2, 1));
			glm::dvec2 P3(poses[0][i] * glm::dvec3(p3, 1));

			angles[i] = atan2(P2.y - points[0].y, P2.x - points[0].x);
		}
		{
			glm::dvec2 P2(poses[0].back() * glm::dvec3(p2, 1));
			glm::dvec2 P3(poses[0].back() * glm::dvec3(p3, 1));

			angles[2] = atan2(P2.y - points[0].y, P2.x - points[0].x);
		}

		// order the angles based on their signs
		int type = 0;
		if (angles[0] < 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[1]) {
			type = 1;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[2]) {
			type = 2;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] >= angles[2]) {
			type = 3;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] < 0 && angles[1] >= 0 && angles[2] >= 0 && (poses[0].size() >= 3 && angles[1] >= angles[2] || poses[0].size() == 2 && angles[1] - angles[0] > M_PI)) {
			type = 4;
			angles[1] -= M_PI * 2;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] < 0 && (poses[0].size() >= 3 && angles[1] < angles[2] || poses[0].size() == 2 && angles[0] - angles[1] > M_PI)) {
			type = 5;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] < angles[2]) {
			type = 6;
			angles[2] -= M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] < 0 && angles[2] >= 0 && angles[0] >= angles[2]) {
			type = 7;
			angles[1] += M_PI * 2;
			angles[2] += M_PI * 2;
		}
		else if (angles[0] >= 0 && angles[1] >= 0 && angles[2] < 0 && angles[0] < angles[1]) {
			type = 8;
			angles[2] += M_PI * 2;
		}

		if (angles[2] < angles[0]) {
			kinematics.invertSpeed();
		}

		// initialize the visited flag
		std::vector<bool> visited(angles.size(), false);
		visited[0] = true;
		int unvisited = 2;

		// run forward until collision is deteted or all the poses are reached
		while (true) {
			try {
				kinematics.stepForward(false, false);	// no collision check
			}
			catch (char* ex) {
				// if only some of the poses are reached before collision, the collision is detected.
				kinematics.clear();
				return length_of_trajectory / length_of_straight;
			}

			// calculate the angle of the driving crank
			double angle = atan2(kinematics.diagram.joints[2]->pos.y - points[0].y, kinematics.diagram.joints[2]->pos.x - points[0].x);

			// update the lengths of the trajectory of the moving body
			for (int i = 0; i < moving_bodies.size(); i++) {
				std::vector<glm::dvec2> next_body_pts = kinematics.diagram.bodies[i]->getActualPoints()[0];
				for (int j = 0; j < next_body_pts.size(); j++) {
					double length = glm::length(next_body_pts[j] - prev_body_pts[i][j]);
					length_of_trajectory += length;
				}
				prev_body_pts[i] = next_body_pts;
			}

			// convert the sign of the angle
			if (type == 1 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 2 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 3 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 4 && angle > 0) {
				angle -= M_PI * 2;
			}
			else if (type == 5 && angle < 0) {
				angle += M_PI * 2;
			}
			else if (type == 6 && angle > angles[0]) {
				angle -= M_PI * 2;
			}
			else if (type == 7 && angle < angles[0]) {
				angle += M_PI * 2;
			}
			else if (type == 8 && angle < 0) {
				angle += M_PI * 2;
			}


			// check if the poses are reached
			for (int i = 0; i < angles.size(); i++) {
				if (visited[i]) continue;

				if (angles[2] >= angles[0]) {
					if (angle >= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
				else {
					if (angle <= angles[i]) {
						visited[i] = true;
						unvisited--;
					}
				}
			}

			// if all the poses are reached without collision, no collision is detected.
			if (unvisited == 0) {
				kinematics.clear();
				return length_of_trajectory / length_of_straight;
			}
		}

		kinematics.clear();
		return length_of_trajectory / length_of_straight;
	}

	/**
	* Calculate the deviation of the link lengths across the poses.
	*
	* @param poses		poses
	* @param points	the coordinates of the joints at the first pose
	* @return			square root of the sum of the devation of the link lengths
	*/
	double LinkageSynthesisWattI::check(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) {
		glm::dvec2 P0 = points[0];
		glm::dvec2 P1 = points[1];
		glm::dvec2 P2 = points[2];
		glm::dvec2 P3 = points[3];
		glm::dvec2 P4 = points[4];
		glm::dvec2 P5 = points[5];
		glm::dvec2 P6 = points[6];
		glm::dvec2 P7 = points[7];
		glm::dvec2 P8 = points[8];
		glm::dvec2 P9 = points[9];

		glm::dvec2 p2(glm::inverse(poses[0][0]) * glm::dvec3(P2, 1));
		glm::dvec2 p3(glm::inverse(poses[0][0]) * glm::dvec3(P3, 1));
		glm::dvec2 p4(glm::inverse(poses[1][0]) * glm::dvec3(P4, 1));
		glm::dvec2 p5(glm::inverse(poses[0][0]) * glm::dvec3(P5, 1));
		glm::dvec2 p6(glm::inverse(poses[1][0]) * glm::dvec3(P6, 1));
		glm::dvec2 p7(glm::inverse(poses[1][0]) * glm::dvec3(P7, 1));
		glm::dvec2 p8(glm::inverse(poses[2][0]) * glm::dvec3(P8, 1));
		glm::dvec2 p9(glm::inverse(poses[2][0]) * glm::dvec3(P9, 1));

		std::vector<double> lengths;
		lengths.push_back(glm::length(P2 - P0));
		lengths.push_back(glm::length(P3 - P1));
		lengths.push_back(glm::length(P3 - P2));
		lengths.push_back(glm::length(P5 - P2));
		lengths.push_back(glm::length(P5 - P3));
		lengths.push_back(glm::length(P4 - P1));
		lengths.push_back(glm::length(P4 - P3));
		lengths.push_back(glm::length(P6 - P4));
		lengths.push_back(glm::length(P6 - P5));
		lengths.push_back(glm::length(P7 - P4));
		lengths.push_back(glm::length(P7 - P6));
		lengths.push_back(glm::length(P8 - P5));
		lengths.push_back(glm::length(P8 - P6));
		lengths.push_back(glm::length(P9 - P8));
		lengths.push_back(glm::length(P9 - P7));

		double ans = 0.0;
		for (int i = 1; i < poses[0].size(); i++) {
			glm::dvec2 P2b(poses[0][i] * glm::dvec3(p2, 1));
			glm::dvec2 P3b(poses[0][i] * glm::dvec3(p3, 1));
			glm::dvec2 P4b(poses[1][i] * glm::dvec3(p4, 1));
			glm::dvec2 P5b(poses[0][i] * glm::dvec3(p5, 1));
			glm::dvec2 P6b(poses[1][i] * glm::dvec3(p6, 1));
			glm::dvec2 P7b(poses[1][i] * glm::dvec3(p7, 1));
			glm::dvec2 P8b(poses[2][i] * glm::dvec3(p8, 1));
			glm::dvec2 P9b(poses[2][i] * glm::dvec3(p9, 1));

			std::vector<double> lengths2;
			lengths2.push_back(glm::length(P2b - P0));
			lengths2.push_back(glm::length(P3b - P1));
			lengths2.push_back(glm::length(P3b - P2b));
			lengths2.push_back(glm::length(P5b - P2b));
			lengths2.push_back(glm::length(P5b - P3b));
			lengths2.push_back(glm::length(P4b - P1));
			lengths2.push_back(glm::length(P4b - P3b));
			lengths2.push_back(glm::length(P6b - P4b));
			lengths2.push_back(glm::length(P6b - P5b));
			lengths2.push_back(glm::length(P7b - P4b));
			lengths2.push_back(glm::length(P7b - P6b));
			lengths2.push_back(glm::length(P8b - P5b));
			lengths2.push_back(glm::length(P8b - P6b));
			lengths2.push_back(glm::length(P9b - P8b));
			lengths2.push_back(glm::length(P9b - P7b));

			for (int i = 0; i < lengths.size(); i++) {
				ans += (lengths[i] - lengths2[i]) * (lengths[i] - lengths2[i]);
			}
		}

		return std::sqrt(ans);
	}

}
