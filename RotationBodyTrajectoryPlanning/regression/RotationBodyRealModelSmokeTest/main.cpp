#include "Adapters/RotationBodyMeshAdapter.h"

#include <RotationBodyTrajectoryPlanning/Alignment/RotationBodyAlignmentSolver.h>
#include <RotationBodyTrajectoryPlanning/Alignment/SimulationBlockPlacementSolver.h>
#include <RotationBodyTrajectoryPlanning/RegionPlanning/SprayBoundaryBuilder.h>
#include <RotationBodyTrajectoryPlanning/RegionPlanning/ToothRegionRecognizer.h>
#include <RotationBodyTrajectoryPlanning/Sectioning/YzSectionExtractor.h>
#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/AutomaticTrajectoryPlanner.h>

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectSession.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <system_error>

#ifndef ROTATION_BODY_SOURCE_ROOT
#error ROTATION_BODY_SOURCE_ROOT must identify the repository source root.
#endif

namespace
{
    namespace app = smrobot::workbench::spray::rotationbody;
    namespace domain = smrobot::spray::rotationbody;

    int failures = 0;

    bool require(bool condition, const std::string& model, const std::string& message)
    {
        if(condition) {
            return true;
        }
        ++failures;
        std::cerr << "FAILED [" << model << "]: " << message << '\n';
        return false;
    }

    const char* errorCodeName(domain::PlanningErrorCode code)
    {
        switch(code) {
        case domain::PlanningErrorCode::None: return "None";
        case domain::PlanningErrorCode::InvalidArgument: return "InvalidArgument";
        case domain::PlanningErrorCode::EmptyMesh: return "EmptyMesh";
        case domain::PlanningErrorCode::NonFiniteGeometry: return "NonFiniteGeometry";
        case domain::PlanningErrorCode::InvalidTriangleIndex: return "InvalidTriangleIndex";
        case domain::PlanningErrorCode::DegenerateGeometry: return "DegenerateGeometry";
        case domain::PlanningErrorCode::InvalidAxisSelection: return "InvalidAxisSelection";
        case domain::PlanningErrorCode::AlignmentFailed: return "AlignmentFailed";
        case domain::PlanningErrorCode::SectionFailed: return "SectionFailed";
        case domain::PlanningErrorCode::NoContour: return "NoContour";
        case domain::PlanningErrorCode::InsufficientRegionData: return "InsufficientRegionData";
        case domain::PlanningErrorCode::InsufficientToothTopData: return "InsufficientToothTopData";
        case domain::PlanningErrorCode::SingularFit: return "SingularFit";
        }
        return "Unknown";
    }

    const char* modeName(domain::BoundaryMode mode)
    {
        return mode == domain::BoundaryMode::MaximumToothTopY
            ? "MaximumToothTopY"
            : "ToothTopEnvelope";
    }

    bool validLabel(domain::RegionLabel label)
    {
        switch(label) {
        case domain::RegionLabel::Unclassified:
        case domain::RegionLabel::ToothTop:
        case domain::RegionLabel::ToothWall:
        case domain::RegionLabel::ToothBottom:
        case domain::RegionLabel::Transition:
            return true;
        }
        return false;
    }

    std::size_t labelIndex(domain::RegionLabel label)
    {
        switch(label) {
        case domain::RegionLabel::Unclassified: return 0;
        case domain::RegionLabel::ToothTop: return 1;
        case domain::RegionLabel::ToothWall: return 2;
        case domain::RegionLabel::ToothBottom: return 3;
        case domain::RegionLabel::Transition: return 4;
        }
        return 0;
    }

    bool isSprayLabel(domain::RegionLabel label)
    {
        return label == domain::RegionLabel::ToothTop ||
            label == domain::RegionLabel::ToothWall ||
            label == domain::RegionLabel::ToothBottom ||
            label == domain::RegionLabel::Transition;
    }

    struct SprayExtents
    {
        bool hasSpray{ false };
        bool hasToothTop{ false };
        double minimumY{ std::numeric_limits<double>::infinity() };
        double maximumY{ -std::numeric_limits<double>::infinity() };
        double minimumZ{ std::numeric_limits<double>::infinity() };
        double maximumZ{ -std::numeric_limits<double>::infinity() };
        double maximumToothTopY{ -std::numeric_limits<double>::infinity() };
    };

    SprayExtents sprayExtents(
        const domain::SectionContour& contour,
        const domain::RegionAssignment& assignment)
    {
        SprayExtents extents;
        for(std::size_t segment = 0; segment < contour.segmentCount(); ++segment) {
            const domain::RegionLabel label = assignment.segmentLabels[segment];
            if(!isSprayLabel(label)) {
                continue;
            }
            extents.hasSpray = true;
            const Eigen::Vector2d points[] = {
                contour.pointsYz[segment],
                contour.pointsYz[(segment + 1) % contour.pointsYz.size()]
            };
            for(const Eigen::Vector2d& point : points) {
                extents.minimumY = std::min(extents.minimumY, point.x());
                extents.maximumY = std::max(extents.maximumY, point.x());
                extents.minimumZ = std::min(extents.minimumZ, point.y());
                extents.maximumZ = std::max(extents.maximumZ, point.y());
                if(label == domain::RegionLabel::ToothTop) {
                    extents.hasToothTop = true;
                    extents.maximumToothTopY = std::max(
                        extents.maximumToothTopY,
                        point.x());
                }
            }
        }
        return extents;
    }

    void validateBoundary(
        const std::string& model,
        const domain::SectionContour& contour,
        const domain::RegionAssignment& assignment,
        domain::BoundaryMode mode,
        double meshScale)
    {
        std::cout << "  stage boundary/" << modeName(mode) << ": begin" << std::endl;
        const auto result = domain::SprayBoundaryBuilder::build(contour, assignment, mode);
        if(!result) {
            require(false, model,
                std::string(modeName(mode)) + " boundary generation failed: " +
                    errorCodeName(result.error.code) + " / " + result.error.message);
            return;
        }

        const domain::SprayBoundary& boundary = result.value;
        const SprayExtents extents = sprayExtents(contour, assignment);
        const double tolerance = std::max({
            meshScale * 1.0e-8,
            contour.toleranceMeters * 8.0,
            1.0e-10 });
        const bool finiteScalars = std::isfinite(boundary.minimumY) &&
            std::isfinite(boundary.maximumY) &&
            std::isfinite(boundary.minimumZ) &&
            std::isfinite(boundary.maximumZ) &&
            std::isfinite(boundary.outerLineSlopeYPerZ) &&
            std::isfinite(boundary.outerLineInterceptY);
        require(finiteScalars, model,
            std::string(modeName(mode)) + " boundary scalars must be finite");
        require(boundary.mode == mode, model,
            std::string(modeName(mode)) + " boundary must preserve its requested mode");
        require(boundary.polygonYz.size() == 4, model,
            std::string(modeName(mode)) + " boundary must contain four polygon vertices");
        for(const Eigen::Vector2d& point : boundary.polygonYz) {
            require(point.allFinite(), model,
                std::string(modeName(mode)) + " polygon vertices must be finite");
        }
        require(extents.hasSpray, model,
            std::string(modeName(mode)) + " succeeded without classified spray segments");
        if(!finiteScalars || !extents.hasSpray) {
            return;
        }

        // The three tight sides must equal the classified segment extents. This rejects
        // the legacy one-percent padding behavior.
        require(std::abs(boundary.minimumY - extents.minimumY) <= tolerance, model,
            std::string(modeName(mode)) + " inner Y side contains artificial padding");
        require(std::abs(boundary.minimumZ - extents.minimumZ) <= tolerance, model,
            std::string(modeName(mode)) + " lower Z side contains artificial padding");
        require(std::abs(boundary.maximumZ - extents.maximumZ) <= tolerance, model,
            std::string(modeName(mode)) + " upper Z side contains artificial padding");

        for(std::size_t segment = 0; segment < contour.segmentCount(); ++segment) {
            if(!isSprayLabel(assignment.segmentLabels[segment])) {
                continue;
            }
            const Eigen::Vector2d points[] = {
                contour.pointsYz[segment],
                contour.pointsYz[(segment + 1) % contour.pointsYz.size()]
            };
            for(const Eigen::Vector2d& point : points) {
                const double outerY = boundary.outerY(point.y());
                require(std::isfinite(outerY), model,
                    std::string(modeName(mode)) + " outer edge evaluation must be finite");
                require(point.x() >= boundary.minimumY - tolerance &&
                    point.x() <= outerY + tolerance &&
                    point.y() >= boundary.minimumZ - tolerance &&
                    point.y() <= boundary.maximumZ + tolerance,
                    model,
                    std::string(modeName(mode)) +
                        " boundary must contain every classified spray segment");
            }
        }

        if(mode == domain::BoundaryMode::MaximumToothTopY) {
            require(extents.hasToothTop, model,
                "MaximumToothTopY succeeded without ToothTop data");
            require(std::abs(boundary.outerLineSlopeYPerZ) <= tolerance &&
                std::abs(boundary.outerLineInterceptY - extents.maximumToothTopY) <= tolerance &&
                std::abs(boundary.maximumY - extents.maximumToothTopY) <= tolerance,
                model,
                "MaximumToothTopY outer side must equal the exact maximum ToothTop Y");
        } else {
            double minimumTopGap = std::numeric_limits<double>::infinity();
            for(std::size_t segment = 0; segment < contour.segmentCount(); ++segment) {
                if(assignment.segmentLabels[segment] != domain::RegionLabel::ToothTop) {
                    continue;
                }
                const Eigen::Vector2d points[] = {
                    contour.pointsYz[segment],
                    contour.pointsYz[(segment + 1) % contour.pointsYz.size()]
                };
                for(const Eigen::Vector2d& point : points) {
                    minimumTopGap = std::min(
                        minimumTopGap,
                        boundary.outerY(point.y()) - point.x());
                }
            }
            require(std::isfinite(minimumTopGap) && minimumTopGap >= -tolerance &&
                minimumTopGap <= tolerance,
                model,
                "ToothTopEnvelope must touch its ToothTop envelope without one-percent expansion");
        }

        const double outerAtMinimumZ = boundary.outerY(boundary.minimumZ);
        const double outerAtMaximumZ = boundary.outerY(boundary.maximumZ);
        require(std::abs(boundary.maximumY -
            std::max(outerAtMinimumZ, outerAtMaximumZ)) <= tolerance,
            model,
            std::string(modeName(mode)) + " maximumY must match its actual outer edge");
        if(boundary.polygonYz.size() == 4) {
            const std::array<Eigen::Vector2d, 4> expectedPolygon = {
                Eigen::Vector2d(boundary.minimumY, boundary.minimumZ),
                Eigen::Vector2d(outerAtMinimumZ, boundary.minimumZ),
                Eigen::Vector2d(outerAtMaximumZ, boundary.maximumZ),
                Eigen::Vector2d(boundary.minimumY, boundary.maximumZ)
            };
            for(std::size_t index = 0; index < expectedPolygon.size(); ++index) {
                require((boundary.polygonYz[index] - expectedPolygon[index]).norm() <= tolerance,
                    model,
                    std::string(modeName(mode)) +
                        " polygon must use the tight classified bounds without padding");
            }
        }
        std::cout << "  boundary " << modeName(mode) << ": success, Y=["
            << boundary.minimumY << ", " << boundary.maximumY << "], Z=["
            << boundary.minimumZ << ", " << boundary.maximumZ << "]\n";
    }

    void testModel(
        const std::filesystem::path& sourceRoot,
        const char* fileName,
        double sectionYawDegrees)
    {
        const std::string model(fileName);
        std::cout << "MODEL " << model << ": begin" << std::endl;
        std::cout << "  stage load: begin" << std::endl;
        const std::string storedPath = std::string("data/Spray420/") + fileName;
        const std::filesystem::path modelPath = sourceRoot / std::filesystem::u8path(storedPath);
        std::error_code fileError;
        if(!require(std::filesystem::is_regular_file(modelPath, fileError) && !fileError,
            model,
            "source STL is missing or inaccessible: " + modelPath.u8string())) {
            return;
        }

        simulation_project::ProjectSession projectSession;
        projectSession.resetNew();
        projectSession.setPath(sourceRoot / "RotationBodyRealModelSmokeTest.sys.json");
        simulation_project::SceneObjectDesc object;
        object.id = "rotation_body_real_model";
        object.name = model;
        object.objectType = "workpiece";
        object.sourcePath = storedPath;
        object.visualScale = 1.0;
        projectSession.document().objects.push_back(object);

        const auto loaded = app::RotationBodyMeshAdapter::load(
            projectSession,
            projectSession.document().objects.back());
        if(!require(loaded.ok(), model,
            std::string("RotationBodyMeshAdapter load failed: ") +
                errorCodeName(loaded.error.code) + " / " + loaded.error.message)) {
            return;
        }
        const domain::TriangleMesh& mesh = loaded.value.mesh;
        const double meshScale = mesh.scale();
        require(meshScale > 1.0e-4 && meshScale < 10.0, model,
            "mesh scale must be a reasonable meter value in (0.1 mm, 10 m)");
        require(!loaded.value.sourceFingerprint.empty(), model,
            "loaded real model must have a source fingerprint");

        std::cout << "  stage alignment: begin" << std::endl;
        const bool simulationBlock = model == u8"\u6a21\u62df\u5757.stl";
        const auto alignment = simulationBlock
            ? domain::SimulationBlockPlacementSolver::solve(
                mesh,
                domain::SignedAxis::PositiveZ,
                domain::SignedAxis::PositiveY,
                0.500)
            : domain::RotationBodyAlignmentSolver::solve(mesh);
        if(!require(alignment.ok(), model,
            std::string("complete-part alignment failed: ") +
                errorCodeName(alignment.error.code) + " / " + alignment.error.message)) {
            return;
        }
        const domain::AlignmentResult& aligned = alignment.value;
        Eigen::Isometry3d sectionYaw = Eigen::Isometry3d::Identity();
        sectionYaw.linear() =
            Eigen::AngleAxisd(
                sectionYawDegrees * std::acos(-1.0) / 180.0,
                Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Isometry3d planningFromMesh =
            sectionYaw * aligned.planningFromMesh;
        const domain::ModelStatistics& statistics = aligned.statistics;
        const bool finiteStatistics = statistics.boundsMinimum.allFinite() &&
            statistics.boundsMaximum.allFinite() &&
            statistics.estimatedAxisInMesh.allFinite() &&
            std::isfinite(statistics.heightMeters) &&
            std::isfinite(statistics.maximumDiameterMeters) &&
            std::isfinite(statistics.minimumDiameterMeters) &&
            std::isfinite(statistics.axisConfidence);
        require(planningFromMesh.matrix().allFinite() &&
            aligned.automaticBaseline.matrix().allFinite() &&
            aligned.bottomAxisCenterInMesh.allFinite() && finiteStatistics,
            model,
            "alignment transform, metadata, and axis center must be finite");
        require(statistics.heightMeters > 1.0e-4 && statistics.heightMeters < 10.0 &&
            statistics.maximumDiameterMeters > 1.0e-4 &&
            statistics.maximumDiameterMeters < 10.0 &&
            statistics.minimumDiameterMeters >= 0.0 &&
            statistics.minimumDiameterMeters <= statistics.maximumDiameterMeters &&
            statistics.axisConfidence >= 0.0 && statistics.axisConfidence <= 1.0,
            model,
            "alignment dimensions and confidence must remain in reasonable meter ranges");

        const double alignmentTolerance = std::max(meshScale * 1.0e-8, 1.0e-9);
        const Eigen::Vector3d plannedAxis =
            planningFromMesh.linear() * statistics.estimatedAxisInMesh;
        require((plannedAxis - Eigen::Vector3d::UnitZ()).norm() <= alignmentTolerance,
            model,
            "automatic alignment must map the directed rotary axis to planning +Z");
        require((planningFromMesh * aligned.bottomAxisCenterInMesh).norm() <=
            alignmentTolerance,
            model,
            "automatic alignment must place the bottom-face axis center at the origin");

        const auto plannedBounds = mesh.bounds(planningFromMesh);
        if(!require(plannedBounds.ok(), model,
            "aligned mesh bounds must be computable")) {
            return;
        }
        require(std::abs(plannedBounds.value.min().z()) <= alignmentTolerance,
            model,
            "aligned mesh Zmin must be approximately zero");

        std::cout << "  stage section: begin" << std::endl;
        const auto section = domain::YzSectionExtractor::extract(
            mesh,
            planningFromMesh);
        if(!require(section.ok(), model,
            std::string("YZ X=0/Y>=0 section failed: ") +
                errorCodeName(section.error.code) + " / " + section.error.message)) {
            return;
        }
        const domain::YzSectionResult& sectionResult = section.value;
        const domain::SectionContour& contour = sectionResult.targetContour;
        require(contour.segmentCount() > 0 && contour.pointsYz.size() >= 2, model,
            "target YZ section must contain a non-empty contour");
        if(!require(contour.points3d.size() == contour.pointsYz.size() &&
            contour.cumulativeArcLength.size() == contour.pointsYz.size(),
            model,
            "YZ, 3D, and cumulative contour arrays must have matching lengths")) {
            return;
        }
        const double sectionTolerance = std::max({
            sectionResult.planeToleranceMeters,
            sectionResult.weldToleranceMeters,
            contour.toleranceMeters,
            1.0e-12 });
        double previousArcLength = -sectionTolerance;
        for(std::size_t index = 0; index < contour.pointsYz.size(); ++index) {
            const Eigen::Vector2d& yz = contour.pointsYz[index];
            const Eigen::Vector3d& point = contour.points3d[index];
            require(yz.allFinite() && point.allFinite(), model,
                "section contour points must be finite");
            require(std::abs(point.x()) <= sectionTolerance, model,
                "section 3D points must lie on planning X=0");
            require(yz.x() >= -sectionTolerance && point.y() >= -sectionTolerance,
                model,
                "target section must remain on the Y>=0 half-plane within tolerance");
            require(std::abs(yz.x() - point.y()) <= sectionTolerance &&
                std::abs(yz.y() - point.z()) <= sectionTolerance,
                model,
                "section YZ and 3D representations must agree");
            const double arcLength = contour.cumulativeArcLength[index];
            require(std::isfinite(arcLength) &&
                arcLength + sectionTolerance >= previousArcLength,
                model,
                "section cumulative arc length must be finite and monotonic");
            previousArcLength = arcLength;
        }

        std::cout << "  stage recognition: begin" << std::endl;
        const auto recognition = domain::ToothRegionRecognizer::recognize(contour);
        if(!require(recognition.ok(), model,
            std::string("four-region recognition failed: ") +
                errorCodeName(recognition.error.code) + " / " + recognition.error.message)) {
            return;
        }
        const domain::RegionAssignment& assignment = recognition.value;
        if(!require(assignment.matches(contour) &&
            assignment.segmentConfidence.size() == contour.segmentCount(),
            model,
            "region labels and confidence must match the contour segment count")) {
            return;
        }
        std::array<std::size_t, 5> labelCounts{};
        for(std::size_t segment = 0; segment < assignment.segmentLabels.size(); ++segment) {
            const domain::RegionLabel label = assignment.segmentLabels[segment];
            require(validLabel(label), model, "recognizer produced an invalid region label");
            if(validLabel(label)) {
                ++labelCounts[labelIndex(label)];
            }
            const double confidence = assignment.segmentConfidence[segment];
            require(std::isfinite(confidence) && confidence >= 0.0 && confidence <= 1.0,
                model,
                "recognizer confidence must be finite and normalized");
        }
        require(labelCounts[0] > 0, model,
            "closed section must retain non-tooth geometry as Unclassified");
        require(labelCounts[1] > 0, model,
            "automatic recognition must identify ToothTop segments");
        require(labelCounts[2] > 0, model,
            "automatic recognition must identify ToothWall segments");
        require(labelCounts[3] > 0, model,
            "automatic recognition must identify ToothBottom segments");
        require(labelCounts[4] > 0, model,
            "automatic recognition must identify Transition segments");
        require(labelCounts[4] < contour.segmentCount() / 3, model,
            "automatic Transition regions must remain localized at tooth-group boundaries");

        std::size_t transitionRunCount = 0;
        std::size_t toothTopRunCount = 0;
        std::size_t wallLikeTopCount = 0;
        std::size_t wallLikeBottomCount = 0;
        std::size_t toothAdjacentWallLikeTransitionCount = 0;
        double maximumTransitionZ = -std::numeric_limits<double>::infinity();
        for(std::size_t segment = 0; segment < assignment.segmentLabels.size(); ++segment) {
            const std::size_t previous = segment == 0
                ? assignment.segmentLabels.size() - 1
                : segment - 1;
            const std::size_t next =
                (segment + 1) % assignment.segmentLabels.size();
            if(assignment.segmentLabels[segment] == domain::RegionLabel::Transition &&
                assignment.segmentLabels[previous] != domain::RegionLabel::Transition) {
                ++transitionRunCount;
            }
            if(assignment.segmentLabels[segment] == domain::RegionLabel::ToothTop &&
                assignment.segmentLabels[previous] != domain::RegionLabel::ToothTop) {
                ++toothTopRunCount;
            }
            const Eigen::Vector2d delta =
                contour.pointsYz[(segment + 1) % contour.pointsYz.size()] -
                contour.pointsYz[segment];
            const double length = delta.norm();
            if(length <= 0.0) {
                continue;
            }
            if(assignment.segmentLabels[segment] == domain::RegionLabel::Transition) {
                maximumTransitionZ = std::max({
                    maximumTransitionZ,
                    contour.pointsYz[segment].y(),
                    contour.pointsYz[(segment + 1) % contour.pointsYz.size()].y() });
            }
            const double wallFacingStrength = std::abs(delta.x()) / length;
            const double radialFacingStrength = std::abs(delta.y()) / length;
            if(assignment.segmentLabels[segment] == domain::RegionLabel::ToothTop &&
                wallFacingStrength > 0.85) {
                ++wallLikeTopCount;
            }
            if(assignment.segmentLabels[segment] == domain::RegionLabel::ToothBottom &&
                radialFacingStrength < 0.62) {
                ++wallLikeBottomCount;
            }
            const Eigen::Vector2d tangent = delta / length;
            const auto continuesAdjacentWall = [&](std::size_t adjacent) {
                if(assignment.segmentLabels[adjacent] != domain::RegionLabel::ToothWall) {
                    return false;
                }
                const Eigen::Vector2d adjacentDelta =
                    contour.pointsYz[(adjacent + 1) % contour.pointsYz.size()] -
                    contour.pointsYz[adjacent];
                const double adjacentLength = adjacentDelta.norm();
                return adjacentLength > 0.0 && std::acos(std::clamp(
                    tangent.dot(adjacentDelta / adjacentLength), -1.0, 1.0)) <= 0.35;
            };
            if(assignment.segmentLabels[segment] == domain::RegionLabel::Transition &&
                wallFacingStrength >= 0.62 &&
                (continuesAdjacentWall(previous) || continuesAdjacentWall(next))) {
                ++toothAdjacentWallLikeTransitionCount;
            }
        }
        require(transitionRunCount >= 1 && transitionRunCount <= 2, model,
            "automatic Transition labels must form only the two tooth-group end regions");
        require(wallLikeTopCount == 0, model,
            "radial wall-facing segments must not be classified as ToothTop");
        require(wallLikeBottomCount == 0, model,
            "sloping wall segments must not be classified as ToothBottom");
        require(toothAdjacentWallLikeTransitionCount == 0, model,
            "the outermost tooth wall must not be absorbed into Transition");

        Eigen::AlignedBox2d contourBounds;
        for(const Eigen::Vector2d& point : contour.pointsYz) {
            contourBounds.extend(point);
        }
        const SprayExtents classifiedExtents = sprayExtents(contour, assignment);
        const double contourRadialRange =
            contourBounds.max().x() - contourBounds.min().x();
        const double contourAxialRange =
            contourBounds.max().y() - contourBounds.min().y();
        const bool feijian = model == "feijian.stl";
        require(classifiedExtents.hasSpray && contourRadialRange > 0.0,
            model,
            "automatic recognition must retain a measurable spray range");
        if(!feijian) {
            require(classifiedExtents.minimumY >=
                contourBounds.min().x() + 0.10 * contourRadialRange,
                model,
                "automatic spray range must not absorb the closed contour's inner wall");
        } else {
            require(toothTopRunCount >= 8,
                model,
                "the fine-tooth fixture must retain its repeated micro-tooth sequence");
            require(classifiedExtents.maximumY <=
                contourBounds.min().x() + 0.02 * contourRadialRange,
                model,
                "the fine-tooth fixture's large outer body must remain Unclassified");
        }
        if(model == "yangjian.STL") {
            require(toothTopRunCount == 3,
                model,
                "the blade fixture must select exactly its three repeated tooth tops");
            require(classifiedExtents.minimumY >=
                contourBounds.min().x() + 0.80 * contourRadialRange,
                model,
                "the blade fixture's rear body must remain Unclassified");
            require(classifiedExtents.maximumZ <=
                contourBounds.min().y() + 0.56 * contourAxialRange,
                model,
                "the blade fixture's upper flange must remain Unclassified");
        }
        if(simulationBlock) {
            require(transitionRunCount == 1,
                model,
                "an edge-mounted tooth group must have only its body-side transition");
            require(maximumTransitionZ <=
                contourBounds.max().y() - 0.05 * contourAxialRange,
                model,
                "the tooth wall connected directly to the top edge must not become Transition");
            const auto dualPlan = domain::AutomaticTrajectoryPlanner::plan(
                contour,
                assignment,
                domain::AutomaticTrajectoryMode::Dual);
            const auto triplePlan = domain::AutomaticTrajectoryPlanner::plan(
                contour,
                assignment,
                domain::AutomaticTrajectoryMode::Triple);
            require(dualPlan.ok() && dualPlan.value.trajectories.size() == 2,
                model,
                "the classified simulation block must support automatic dual trajectories");
            require(triplePlan.ok() && triplePlan.value.trajectories.size() == 3,
                model,
                "the classified simulation block must support automatic triple trajectories");
            if(dualPlan && dualPlan.value.trajectories.size() == 2) {
                require(dualPlan.value.trajectories[0].tiltRadians >= 0.0 &&
                        dualPlan.value.trajectories[1].tiltRadians <= 0.0 &&
                        std::abs(dualPlan.value.trajectories[0].tiltRadians) <=
                            dualPlan.value.dualTiltLimitRadians + 1.0e-12 &&
                        std::abs(dualPlan.value.trajectories[1].tiltRadians) <=
                            dualPlan.value.dualTiltLimitRadians + 1.0e-12,
                    model,
                    "dual upper/lower tilts must retain their signs and coverage limit");
            }
            if(triplePlan && triplePlan.value.trajectories.size() == 3) {
                require(triplePlan.value.trajectories[0].tiltRadians >
                            triplePlan.value.trajectories[1].tiltRadians,
                    model,
                    "triple upper and lower wall trajectories must retain distinct tilts");
            }
        }
        const std::string fourthFixtureSuffix = "4.stl";
        if(model.size() >= fourthFixtureSuffix.size() &&
            model.compare(
                model.size() - fourthFixtureSuffix.size(),
                fourthFixtureSuffix.size(),
                fourthFixtureSuffix) == 0) {
            require(classifiedExtents.maximumY <=
                contourBounds.max().x() - 0.25 * contourRadialRange,
                model,
                "repeated teeth must be selected inside the larger outer flange");
        }

        std::cout << std::fixed << std::setprecision(6)
            << "MODEL " << model
            << ": vertices=" << mesh.vertexCount()
            << ", triangles=" << mesh.triangleCount()
            << ", scale=" << meshScale << " m"
            << ", height=" << statistics.heightMeters << " m"
            << ", maxDiameter=" << statistics.maximumDiameterMeters << " m"
            << ", minDiameter=" << statistics.minimumDiameterMeters << " m"
            << ", confidence=" << statistics.axisConfidence
            << ", zMin=" << plannedBounds.value.min().z() << " m"
            << ", contourPoints=" << contour.pointsYz.size()
            << ", contourSegments=" << contour.segmentCount()
            << ", candidateContours=" << sectionResult.candidateContours.size()
            << ", contourY=[" << contourBounds.min().x()
            << ", " << contourBounds.max().x() << "]"
            << ", contourZ=[" << contourBounds.min().y()
            << ", " << contourBounds.max().y() << "]"
            << '\n'
            << "  labels: Unclassified=" << labelCounts[0]
            << ", ToothTop=" << labelCounts[1]
            << ", ToothWall=" << labelCounts[2]
            << ", ToothBottom=" << labelCounts[3]
            << ", Transition=" << labelCounts[4] << '\n';
        for(const std::string& diagnostic : recognition.diagnostics) {
            std::cout << "  recognition: " << diagnostic << '\n';
        }

        validateBoundary(
            model,
            contour,
            assignment,
            domain::BoundaryMode::MaximumToothTopY,
            meshScale);
        validateBoundary(
            model,
            contour,
            assignment,
            domain::BoundaryMode::ToothTopEnvelope,
            meshScale);
    }
    int selectedModelIndex(
        const std::string& argument,
        const std::array<const char*, 7>& modelNames)
    {
        if(argument.size() == 1 && argument.front() >= '1' && argument.front() <= '7') {
            return argument.front() - '1';
        }
        for(std::size_t index = 0; index < modelNames.size(); ++index) {
            if(argument == modelNames[index]) {
                return static_cast<int>(index);
            }
            const std::string suffix = std::to_string(index + 1) + ".stl";
            if(argument.size() >= suffix.size() &&
                argument.compare(argument.size() - suffix.size(), suffix.size(), suffix) == 0) {
                return static_cast<int>(index);
            }
        }
        return -1;
    }
}

int main(int argc, char** argv)
{
    const std::filesystem::path sourceRoot =
        std::filesystem::u8path(ROTATION_BODY_SOURCE_ROOT);
    const std::array<const char*, 7> modelNames = {
        u8"篦齿试验件1.stl",
        u8"篦齿试验件2.stl",
        u8"篦齿试验件3.stl",
        u8"篦齿试验件4.stl",
        u8"yangjian.STL",
        u8"模拟块.stl",
        u8"feijian.stl"
    };

    int firstModel = 0;
    int modelCount = static_cast<int>(modelNames.size());
    if(argc > 3) {
        std::cerr << "Usage: RotationBodyRealModelSmokeTest [1-7|model-file-name] [section-yaw-degrees]" << std::endl;
        return 2;
    }
    if(argc >= 2) {
        firstModel = selectedModelIndex(argv[1], modelNames);
        if(firstModel < 0) {
            std::cerr << "Unknown model selector: " << argv[1] << '\n'
                << "Usage: RotationBodyRealModelSmokeTest [1-7|model-file-name] [section-yaw-degrees]" << std::endl;
            return 2;
        }
        modelCount = 1;
    }

    double sectionYawDegrees = 0.0;
    if(argc == 3) {
        try {
            sectionYawDegrees = std::stod(argv[2]);
        } catch(const std::exception&) {
            std::cerr << "Invalid section yaw in degrees: " << argv[2] << std::endl;
            return 2;
        }
    }

    for(int offset = 0; offset < modelCount; ++offset) {
        const char* modelName = modelNames[static_cast<std::size_t>(firstModel + offset)];
        const double modelYawDegrees = argc == 3
            ? sectionYawDegrees
            : (std::string(modelName) == "feijian.stl" ? 45.0 : 0.0);
        try {
            testModel(sourceRoot, modelName, modelYawDegrees);
        } catch(const std::exception& error) {
            ++failures;
            std::cerr << "FAILED [" << modelName << "]: unexpected exception: "
                << error.what() << '\n';
        } catch(...) {
            ++failures;
            std::cerr << "FAILED [" << modelName << "]: unexpected non-standard exception\n";
        }
    }

    if(failures == 0) {
        std::cout << "Rotation-body real-model smoke test completed without failures."
            << std::endl;
    } else {
        std::cerr << "Rotation-body real-model smoke test failures: " << failures
            << std::endl;
    }
    return failures == 0 ? 0 : 1;
}
