#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include "Solution.h"
#include "BBox.h"
#include "KinematicUtils.h"
#include "Kinematics.h"

namespace kinematics {

	class LinkageSynthesis {
	protected:
		std::vector<Object25D> fixed_bodies;
		std::pair<double, double> sigmas;
		bool avoid_branch_defect;
		double min_transmission_angle;
		double min_link_length;
		std::vector<double> weights;

	protected:
		LinkageSynthesis() {}

	public:
		static void calculateStatistics(const std::vector<double>& values, double& mean, double& sd);
		static bool compare(const Solution& s1, const Solution& s2);
		static std::vector<std::vector<glm::dmat3x3>> perturbPoses(const std::vector<std::vector<glm::dmat3x3>>& poses, std::pair<double, double>& sigmas, double& position_error, double& orientation_error);
		static std::vector<glm::dvec2> enlargePolygon(const std::vector<glm::dvec2>& polygon, const glm::dvec2& center, double scale);
		static void createDistanceMapForLinkageRegion(const std::vector<glm::dvec2>& linkage_region_pts, double scale, BBox& dist_map_bbox, cv::Mat& dist_map);
		void particleFilter(std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& moving_bodies, int num_particles, int num_iterations, bool record_file);
		void resample(std::vector<Solution> particles, int N, std::vector<Solution>& resampled_particles, double max_cost);

		virtual void calculateSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, int num_samples, const std::vector<Object25D>& moving_bodies, std::vector<Solution>& solutions) = 0;
		virtual bool optimizeCandidate(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const BBox& bbox, std::vector<glm::dvec2>& points) = 0;
		virtual Solution findBestSolution(const std::vector<std::vector<glm::dmat3x3>>& poses, std::vector<Solution>& solutions, const std::vector<glm::dvec2>& linkage_region_pts, const cv::Mat& dist_map, const BBox& dist_map_bbox, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& moving_bodies, int num_particles, int num_iterations, bool record_file) = 0;
		virtual bool checkHardConstraints(std::vector<glm::dvec2>& points, const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& linkage_region_pts, const std::vector<glm::dvec2>& linkage_avoidance_pts, const std::vector<Object25D>& moving_bodies, double simulation_speed) = 0;
		virtual int getType(const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkOrderDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkBranchDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual bool checkCircuitDefect(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points) = 0;
		virtual Kinematics constructKinematics(const std::vector<std::vector<glm::dmat3x3>>& poses, const std::vector<glm::dvec2>& points, const std::vector<Object25D>& moving_body, bool connect_joints, const std::vector<Object25D>& fixed_bodies, std::vector<glm::dvec2>& connected_pt) = 0;
		virtual void updateMovingBodies(Kinematics& kin, const std::vector<Object25D>& moving_body) = 0;
		virtual double calculateCost(Solution& solution, const std::vector<Object25D>& moving_bodies, const cv::Mat& dist_map, const BBox& dist_map_bbox) = 0;
	};

}