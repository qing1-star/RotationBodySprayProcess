#include "RotationBodyPlanningDraftStore.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        using Json = nlohmann::json;

        Json vector2Json(const Eigen::Vector2d& value)
        {
            return Json::array({ value.x(), value.y() });
        }

        Json vector3Json(const Eigen::Vector3d& value)
        {
            return Json::array({ value.x(), value.y(), value.z() });
        }

        Eigen::Vector2d vector2(const Json& value)
        {
            if(!value.is_array() || value.size() != 2) {
                throw std::runtime_error("Expected a two-component vector.");
            }
            return { value.at(0).get<double>(), value.at(1).get<double>() };
        }

        Eigen::Vector3d vector3(const Json& value)
        {
            if(!value.is_array() || value.size() != 3) {
                throw std::runtime_error("Expected a three-component vector.");
            }
            return {
                value.at(0).get<double>(),
                value.at(1).get<double>(),
                value.at(2).get<double>()
            };
        }

        Json transformJson(const Eigen::Isometry3d& transform)
        {
            Json values = Json::array();
            for(int row = 0; row < 4; ++row) {
                for(int column = 0; column < 4; ++column) {
                    values.push_back(transform.matrix()(row, column));
                }
            }
            return values;
        }

        Eigen::Isometry3d transform(const Json& value)
        {
            if(!value.is_array() || value.size() != 16) {
                throw std::runtime_error("Expected a 4x4 transform matrix.");
            }
            Eigen::Matrix4d matrix;
            for(int row = 0; row < 4; ++row) {
                for(int column = 0; column < 4; ++column) {
                    matrix(row, column) = value.at(row * 4 + column).get<double>();
                }
            }
            if(!matrix.allFinite() ||
                !matrix.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1.0e-9)) {
                throw std::runtime_error("Transform matrix is not finite or affine.");
            }
            Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
            result.matrix() = matrix;
            const Eigen::Matrix3d orthogonality = result.linear().transpose() * result.linear();
            if(!orthogonality.isApprox(Eigen::Matrix3d::Identity(), 1.0e-8) ||
                std::abs(result.linear().determinant() - 1.0) > 1.0e-8) {
                throw std::runtime_error("Transform matrix is not rigid.");
            }
            return result;
        }

        const char* planningObjectTypeName(domain::PlanningObjectType value)
        {
            return value == domain::PlanningObjectType::SimulationBlock
                ? "simulationBlock"
                : "completePart";
        }

        domain::PlanningObjectType planningObjectType(const std::string& value)
        {
            if(value == "completePart") {
                return domain::PlanningObjectType::CompletePart;
            }
            if(value == "simulationBlock") {
                return domain::PlanningObjectType::SimulationBlock;
            }
            throw std::runtime_error("Unknown planning object type.");
        }

        const char* signedAxisName(domain::SignedAxis value)
        {
            switch(value) {
            case domain::SignedAxis::PositiveX: return "+X";
            case domain::SignedAxis::NegativeX: return "-X";
            case domain::SignedAxis::PositiveY: return "+Y";
            case domain::SignedAxis::NegativeY: return "-Y";
            case domain::SignedAxis::PositiveZ: return "+Z";
            case domain::SignedAxis::NegativeZ: return "-Z";
            }
            return "+Z";
        }

        domain::SignedAxis signedAxis(const std::string& value)
        {
            if(value == "+X") return domain::SignedAxis::PositiveX;
            if(value == "-X") return domain::SignedAxis::NegativeX;
            if(value == "+Y") return domain::SignedAxis::PositiveY;
            if(value == "-Y") return domain::SignedAxis::NegativeY;
            if(value == "+Z") return domain::SignedAxis::PositiveZ;
            if(value == "-Z") return domain::SignedAxis::NegativeZ;
            throw std::runtime_error("Unknown signed axis.");
        }

        const char* stageName(domain::PlanningStage value)
        {
            switch(value) {
            case domain::PlanningStage::NoModel: return "noModel";
            case domain::PlanningStage::ModelLoaded: return "modelLoaded";
            case domain::PlanningStage::FrameConfirmed: return "frameConfirmed";
            case domain::PlanningStage::SectionReady: return "sectionReady";
            case domain::PlanningStage::RegionsReady: return "regionsReady";
            case domain::PlanningStage::SprayBoundaryConfirmed: return "sprayBoundaryConfirmed";
            }
            return "noModel";
        }

        domain::PlanningStage stage(const std::string& value)
        {
            if(value == "noModel") return domain::PlanningStage::NoModel;
            if(value == "modelLoaded") return domain::PlanningStage::ModelLoaded;
            if(value == "frameConfirmed") return domain::PlanningStage::FrameConfirmed;
            if(value == "sectionReady") return domain::PlanningStage::SectionReady;
            if(value == "regionsReady") return domain::PlanningStage::RegionsReady;
            if(value == "sprayBoundaryConfirmed") {
                return domain::PlanningStage::SprayBoundaryConfirmed;
            }
            throw std::runtime_error("Unknown planning stage.");
        }

        const char* regionLabelName(domain::RegionLabel value)
        {
            switch(value) {
            case domain::RegionLabel::Unclassified: return "unclassified";
            case domain::RegionLabel::ToothTop: return "toothTop";
            case domain::RegionLabel::ToothWall: return "toothWall";
            case domain::RegionLabel::ToothBottom: return "toothBottom";
            case domain::RegionLabel::Transition: return "transition";
            }
            return "unclassified";
        }

        domain::RegionLabel regionLabel(const std::string& value)
        {
            if(value == "unclassified") return domain::RegionLabel::Unclassified;
            if(value == "toothTop") return domain::RegionLabel::ToothTop;
            if(value == "toothWall") return domain::RegionLabel::ToothWall;
            if(value == "toothBottom") return domain::RegionLabel::ToothBottom;
            if(value == "transition") return domain::RegionLabel::Transition;
            throw std::runtime_error("Unknown region label.");
        }

        const char* boundaryModeName(domain::BoundaryMode value)
        {
            return value == domain::BoundaryMode::ToothTopEnvelope
                ? "toothTopEnvelope"
                : "maximumToothTopY";
        }

        domain::BoundaryMode boundaryMode(const std::string& value)
        {
            if(value == "maximumToothTopY") return domain::BoundaryMode::MaximumToothTopY;
            if(value == "toothTopEnvelope") return domain::BoundaryMode::ToothTopEnvelope;
            throw std::runtime_error("Unknown boundary mode.");
        }

        const char* publishFrameName(PublishFrame value)
        {
            return value == PublishFrame::PlanningLocalFrame ? "planningLocal" : "base";
        }

        PublishFrame publishFrame(const std::string& value)
        {
            if(value == "base") return PublishFrame::BaseFrame;
            if(value == "planningLocal") return PublishFrame::PlanningLocalFrame;
            throw std::runtime_error("Unknown publish frame.");
        }

        Json statisticsJson(const domain::ModelStatistics& statistics)
        {
            return {
                { "vertexCount", statistics.vertexCount },
                { "triangleCount", statistics.triangleCount },
                { "boundsMinimum", vector3Json(statistics.boundsMinimum) },
                { "boundsMaximum", vector3Json(statistics.boundsMaximum) },
                { "estimatedAxisInMesh", vector3Json(statistics.estimatedAxisInMesh) },
                { "heightMeters", statistics.heightMeters },
                { "maximumDiameterMeters", statistics.maximumDiameterMeters },
                { "minimumDiameterMeters", statistics.minimumDiameterMeters },
                { "axisConfidence", statistics.axisConfidence }
            };
        }

        domain::ModelStatistics statistics(const Json& value)
        {
            domain::ModelStatistics result;
            result.vertexCount = value.at("vertexCount").get<std::size_t>();
            result.triangleCount = value.at("triangleCount").get<std::size_t>();
            result.boundsMinimum = vector3(value.at("boundsMinimum"));
            result.boundsMaximum = vector3(value.at("boundsMaximum"));
            result.estimatedAxisInMesh = vector3(value.at("estimatedAxisInMesh"));
            result.heightMeters = value.at("heightMeters").get<double>();
            result.maximumDiameterMeters = value.at("maximumDiameterMeters").get<double>();
            result.minimumDiameterMeters = value.at("minimumDiameterMeters").get<double>();
            result.axisConfidence = value.at("axisConfidence").get<double>();
            return result;
        }

        Json sectionJson(const domain::SectionContour& section)
        {
            Json pointsYz = Json::array();
            for(const Eigen::Vector2d& point : section.pointsYz) pointsYz.push_back(vector2Json(point));
            Json points3d = Json::array();
            for(const Eigen::Vector3d& point : section.points3d) points3d.push_back(vector3Json(point));
            return {
                { "pointsYz", std::move(pointsYz) },
                { "points3d", std::move(points3d) },
                { "cumulativeArcLength", section.cumulativeArcLength },
                { "closed", section.closed },
                { "toleranceMeters", section.toleranceMeters },
                { "diagnostics", section.diagnostics }
            };
        }

        domain::SectionContour section(const Json& value)
        {
            domain::SectionContour result;
            for(const Json& point : value.at("pointsYz")) result.pointsYz.push_back(vector2(point));
            for(const Json& point : value.at("points3d")) result.points3d.push_back(vector3(point));
            result.cumulativeArcLength = value.at("cumulativeArcLength").get<std::vector<double>>();
            result.closed = value.at("closed").get<bool>();
            result.toleranceMeters = value.at("toleranceMeters").get<double>();
            result.diagnostics = value.value("diagnostics", std::vector<std::string>{});
            if(result.pointsYz.size() < 2 || result.points3d.size() != result.pointsYz.size()) {
                throw std::runtime_error("Stored section contour dimensions are invalid.");
            }
            return result;
        }

        Json regionsJson(const domain::RegionAssignment& regions)
        {
            Json labels = Json::array();
            for(domain::RegionLabel label : regions.segmentLabels) labels.push_back(regionLabelName(label));
            return {
                { "labels", std::move(labels) },
                { "confidence", regions.segmentConfidence },
                { "diagnostics", regions.diagnostics }
            };
        }

        domain::RegionAssignment regions(const Json& value)
        {
            domain::RegionAssignment result;
            for(const Json& label : value.at("labels")) {
                result.segmentLabels.push_back(regionLabel(label.get<std::string>()));
            }
            result.segmentConfidence = value.at("confidence").get<std::vector<double>>();
            result.diagnostics = value.value("diagnostics", std::vector<std::string>{});
            if(!result.segmentConfidence.empty() &&
                result.segmentLabels.size() != result.segmentConfidence.size()) {
                throw std::runtime_error("Stored region dimensions are invalid.");
            }
            return result;
        }

        Json commandJson(const domain::RegionOverrideCommand& command)
        {
            return {
                { "minimum", vector2Json(command.rectangle.minimum) },
                { "maximum", vector2Json(command.rectangle.maximum) },
                { "label", regionLabelName(command.label) }
            };
        }

        domain::RegionOverrideCommand command(const Json& value)
        {
            domain::RegionOverrideCommand result;
            result.rectangle.minimum = vector2(value.at("minimum"));
            result.rectangle.maximum = vector2(value.at("maximum"));
            result.label = regionLabel(value.at("label").get<std::string>());
            if(!result.rectangle.isFinite()) {
                throw std::runtime_error("Stored edit rectangle is not finite.");
            }
            return result;
        }

        Json boundaryJson(const domain::SprayBoundary& boundary)
        {
            Json polygon = Json::array();
            for(const Eigen::Vector2d& point : boundary.polygonYz) polygon.push_back(vector2Json(point));
            return {
                { "mode", boundaryModeName(boundary.mode) },
                { "polygonYz", std::move(polygon) },
                { "minimumY", boundary.minimumY },
                { "maximumY", boundary.maximumY },
                { "minimumZ", boundary.minimumZ },
                { "maximumZ", boundary.maximumZ },
                { "outerLineSlopeYPerZ", boundary.outerLineSlopeYPerZ },
                { "outerLineInterceptY", boundary.outerLineInterceptY }
            };
        }

        domain::SprayBoundary boundary(const Json& value)
        {
            domain::SprayBoundary result;
            result.mode = boundaryMode(value.at("mode").get<std::string>());
            for(const Json& point : value.at("polygonYz")) result.polygonYz.push_back(vector2(point));
            result.minimumY = value.at("minimumY").get<double>();
            result.maximumY = value.at("maximumY").get<double>();
            result.minimumZ = value.at("minimumZ").get<double>();
            result.maximumZ = value.at("maximumZ").get<double>();
            result.outerLineSlopeYPerZ = value.at("outerLineSlopeYPerZ").get<double>();
            result.outerLineInterceptY = value.at("outerLineInterceptY").get<double>();
            if(result.polygonYz.size() < 3) {
                throw std::runtime_error("Stored spray boundary is incomplete.");
            }
            return result;
        }

        Json parametersJson(const RotationBodyAlgorithmParameters& parameters)
        {
            return {
                { "section", {
                    { "relativePlaneTolerance", parameters.section.relativePlaneTolerance },
                    { "relativeWeldTolerance", parameters.section.relativeWeldTolerance },
                    { "minimumToleranceMeters", parameters.section.minimumToleranceMeters }
                } },
                { "region", {
                    { "minimumResampleCount", parameters.region.minimumResampleCount },
                    { "maximumResampleCount", parameters.region.maximumResampleCount },
                    { "smoothingWindowFraction", parameters.region.smoothingWindowFraction },
                    { "minimumRadialProminenceFraction", parameters.region.minimumRadialProminenceFraction },
                    { "axialSurfaceThreshold", parameters.region.axialSurfaceThreshold },
                    { "minimumConfidence", parameters.region.minimumConfidence },
                    { "transitionExtentFraction", parameters.region.transitionExtentFraction }
                } }
            };
        }

        RotationBodyAlgorithmParameters parameters(const Json& value)
        {
            RotationBodyAlgorithmParameters result;
            const Json& sectionValue = value.at("section");
            result.section.relativePlaneTolerance =
                sectionValue.at("relativePlaneTolerance").get<double>();
            result.section.relativeWeldTolerance =
                sectionValue.at("relativeWeldTolerance").get<double>();
            result.section.minimumToleranceMeters =
                sectionValue.at("minimumToleranceMeters").get<double>();
            const Json& regionValue = value.at("region");
            result.region.minimumResampleCount = regionValue.at("minimumResampleCount").get<std::size_t>();
            result.region.maximumResampleCount = regionValue.at("maximumResampleCount").get<std::size_t>();
            result.region.smoothingWindowFraction = regionValue.at("smoothingWindowFraction").get<double>();
            result.region.minimumRadialProminenceFraction =
                regionValue.at("minimumRadialProminenceFraction").get<double>();
            result.region.axialSurfaceThreshold = regionValue.at("axialSurfaceThreshold").get<double>();
            result.region.minimumConfidence = regionValue.at("minimumConfidence").get<double>();
            result.region.transitionExtentFraction = regionValue.at("transitionExtentFraction").get<double>();
            return result;
        }

        const char* calibrationModeName(domain::CalibrationMode mode)
        {
            return mode == domain::CalibrationMode::Circle2d ? "circle2d" : "cylinder3d";
        }

        domain::CalibrationMode calibrationMode(const std::string& value)
        {
            if(value == "cylinder3d") return domain::CalibrationMode::Cylinder3d;
            if(value == "circle2d") return domain::CalibrationMode::Circle2d;
            throw std::runtime_error("Unknown calibration mode.");
        }

        Json calibrationRowJson(const CalibrationCoordinateRow& row)
        {
            Json result = Json::array();
            for(const std::optional<double>& value : row) {
                result.push_back(value ? Json(*value) : Json(nullptr));
            }
            return result;
        }

        CalibrationCoordinateRow calibrationRow(const Json& value)
        {
            if(!value.is_array() || value.size() != 3) {
                throw std::runtime_error("Expected a three-coordinate calibration row.");
            }
            CalibrationCoordinateRow result;
            for(std::size_t index = 0; index < result.size(); ++index) {
                if(!value.at(index).is_null()) {
                    result[index] = value.at(index).get<double>();
                }
            }
            return result;
        }

        Json calibrationFitJson(const domain::CalibrationAxisFit& fit)
        {
            Json points = Json::array();
            for(const Eigen::Vector3d& point : fit.sourcePointsBaseMeters) {
                points.push_back(vector3Json(point));
            }
            return {
                { "mode", calibrationModeName(fit.mode) },
                { "axisPointBaseMeters", vector3Json(fit.axisPointBaseMeters) },
                { "axisDirectionBase", vector3Json(fit.axisDirectionBase) },
                { "radiusMeters", fit.radiusMeters },
                { "rmsResidualMeters", fit.rmsResidualMeters },
                { "maximumResidualMeters", fit.maximumResidualMeters },
                { "iterations", fit.iterations },
                { "lowAxialSpan", fit.lowAxialSpan },
                { "residualsMeters", fit.residualsMeters },
                { "sourcePointsBaseMeters", std::move(points) }
            };
        }

        domain::CalibrationAxisFit calibrationFit(const Json& value)
        {
            domain::CalibrationAxisFit result;
            result.mode = calibrationMode(value.at("mode").get<std::string>());
            result.axisPointBaseMeters = vector3(value.at("axisPointBaseMeters"));
            result.axisDirectionBase = vector3(value.at("axisDirectionBase"));
            result.radiusMeters = value.at("radiusMeters").get<double>();
            result.rmsResidualMeters = value.at("rmsResidualMeters").get<double>();
            result.maximumResidualMeters = value.at("maximumResidualMeters").get<double>();
            result.iterations = value.at("iterations").get<int>();
            result.lowAxialSpan = value.at("lowAxialSpan").get<bool>();
            result.residualsMeters = value.at("residualsMeters").get<std::vector<double>>();
            for(const Json& point : value.at("sourcePointsBaseMeters")) {
                result.sourcePointsBaseMeters.push_back(vector3(point));
            }
            return result;
        }

        Json calibrationFrameJson(const domain::WorkpieceFrameCalibration& frame)
        {
            return {
                { "baseFromPlanning", transformJson(frame.baseFromPlanning) },
                { "translationMeters", vector3Json(
                    frame.baseFromPlanningComponents.translationMeters) },
                { "rollPitchYawRadians", vector3Json(
                    frame.baseFromPlanningComponents.rollPitchYawRadians) },
                { "topReferenceBaseMeters", vector3Json(frame.topReferenceBaseMeters) },
                { "topCenterBaseMeters", vector3Json(frame.topCenterBaseMeters) },
                { "baseOriginBaseMeters", vector3Json(frame.baseOriginBaseMeters) },
                { "yDirectionStartBaseMeters", vector3Json(
                    frame.yDirectionStartBaseMeters) },
                { "yDirectionEndBaseMeters", vector3Json(frame.yDirectionEndBaseMeters) },
                { "xAxisBase", vector3Json(frame.xAxisBase) },
                { "yAxisBase", vector3Json(frame.yAxisBase) },
                { "zAxisBase", vector3Json(frame.zAxisBase) },
                { "abbQuaternionWxyz", Json::array({
                    frame.abbQuaternionWxyz.x(),
                    frame.abbQuaternionWxyz.y(),
                    frame.abbQuaternionWxyz.z(),
                    frame.abbQuaternionWxyz.w() }) },
                { "workpieceHeightMeters", frame.workpieceHeightMeters },
                { "fittedRadiusMeters", frame.fittedRadiusMeters },
                { "yDirectionDistanceMeters", frame.yDirectionDistanceMeters },
                { "shortYDirectionBaseline", frame.shortYDirectionBaseline }
            };
        }

        domain::WorkpieceFrameCalibration calibrationFrame(const Json& value)
        {
            domain::WorkpieceFrameCalibration result;
            result.baseFromPlanning = transform(value.at("baseFromPlanning"));
            result.baseFromPlanningComponents.translationMeters =
                vector3(value.at("translationMeters"));
            result.baseFromPlanningComponents.rollPitchYawRadians =
                vector3(value.at("rollPitchYawRadians"));
            result.topReferenceBaseMeters = vector3(value.at("topReferenceBaseMeters"));
            result.topCenterBaseMeters = vector3(value.at("topCenterBaseMeters"));
            result.baseOriginBaseMeters = vector3(value.at("baseOriginBaseMeters"));
            result.yDirectionStartBaseMeters =
                vector3(value.at("yDirectionStartBaseMeters"));
            result.yDirectionEndBaseMeters = vector3(value.at("yDirectionEndBaseMeters"));
            result.xAxisBase = vector3(value.at("xAxisBase"));
            result.yAxisBase = vector3(value.at("yAxisBase"));
            result.zAxisBase = vector3(value.at("zAxisBase"));
            const Json& quaternion = value.at("abbQuaternionWxyz");
            if(!quaternion.is_array() || quaternion.size() != 4) {
                throw std::runtime_error("Stored ABB quaternion is invalid.");
            }
            result.abbQuaternionWxyz = {
                quaternion.at(0).get<double>(),
                quaternion.at(1).get<double>(),
                quaternion.at(2).get<double>(),
                quaternion.at(3).get<double>()
            };
            result.workpieceHeightMeters = value.at("workpieceHeightMeters").get<double>();
            result.fittedRadiusMeters = value.at("fittedRadiusMeters").get<double>();
            result.yDirectionDistanceMeters =
                value.at("yDirectionDistanceMeters").get<double>();
            result.shortYDirectionBaseline = value.at("shortYDirectionBaseline").get<bool>();
            return result;
        }

        template<std::size_t Size>
        Json calibrationRowsJson(
            const std::array<CalibrationCoordinateRow, Size>& rows)
        {
            Json result = Json::array();
            for(const CalibrationCoordinateRow& row : rows) {
                result.push_back(calibrationRowJson(row));
            }
            return result;
        }

        template<std::size_t Size>
        std::array<CalibrationCoordinateRow, Size> calibrationRows(const Json& value)
        {
            if(!value.is_array() || value.size() != Size) {
                throw std::runtime_error("Stored calibration table dimensions are invalid.");
            }
            std::array<CalibrationCoordinateRow, Size> result;
            for(std::size_t index = 0; index < Size; ++index) {
                result[index] = calibrationRow(value.at(index));
            }
            return result;
        }

        Json calibrationJson(const WorkpieceCalibrationWorkspace& workspace)
        {
            Json result = {
                { "cylinderRows", calibrationRowsJson(workspace.cylinderRows) },
                { "circleRows", calibrationRowsJson(workspace.circleRows) },
                { "topReference", calibrationRowJson(workspace.topReference) },
                { "yDirectionStart", calibrationRowJson(workspace.yDirectionStart) },
                { "yDirectionEnd", calibrationRowJson(workspace.yDirectionEnd) },
                { "workpieceHeightMeters", workspace.workpieceHeightMeters },
                { "activeMode", calibrationModeName(workspace.activeMode) },
                { "axisSource", calibrationModeName(workspace.axisSource) },
                { "showCylinder", workspace.showCylinder },
                { "showCircle", workspace.showCircle }
            };
            if(workspace.cylinderFit) {
                result["cylinderFit"] = calibrationFitJson(*workspace.cylinderFit);
            }
            if(workspace.circleFit) {
                result["circleFit"] = calibrationFitJson(*workspace.circleFit);
            }
            if(workspace.frame) {
                result["frame"] = calibrationFrameJson(*workspace.frame);
            }
            return result;
        }

        WorkpieceCalibrationWorkspace calibration(const Json& value)
        {
            WorkpieceCalibrationWorkspace result;
            result.cylinderRows = calibrationRows<12>(value.at("cylinderRows"));
            result.circleRows = calibrationRows<6>(value.at("circleRows"));
            result.topReference = calibrationRow(value.at("topReference"));
            result.yDirectionStart = calibrationRow(value.at("yDirectionStart"));
            result.yDirectionEnd = calibrationRow(value.at("yDirectionEnd"));
            result.workpieceHeightMeters = value.at("workpieceHeightMeters").get<double>();
            result.activeMode = calibrationMode(value.at("activeMode").get<std::string>());
            result.axisSource = calibrationMode(value.at("axisSource").get<std::string>());
            result.showCylinder = value.at("showCylinder").get<bool>();
            result.showCircle = value.at("showCircle").get<bool>();
            if(value.contains("cylinderFit")) {
                result.cylinderFit = calibrationFit(value.at("cylinderFit"));
            }
            if(value.contains("circleFit")) {
                result.circleFit = calibrationFit(value.at("circleFit"));
            }
            if(value.contains("frame")) {
                result.frame = calibrationFrame(value.at("frame"));
            }
            return result;
        }

        const char* workflowName(RotationBodyWorkflow value)
        {
            return value == RotationBodyWorkflow::SectionRegionPlanning
                ? "sectionRegionPlanning"
                : "modelTransform";
        }

        RotationBodyWorkflow workflow(const std::string& value)
        {
            if(value == "modelTransform") return RotationBodyWorkflow::ModelTransform;
            if(value == "sectionRegionPlanning") {
                return RotationBodyWorkflow::SectionRegionPlanning;
            }
            throw std::runtime_error("Unknown rotation-body workflow.");
        }

        const char* mainViewModeName(RotationBodyMainViewMode value)
        {
            return value == RotationBodyMainViewMode::Section ? "section" : "scene3d";
        }

        RotationBodyMainViewMode mainViewMode(const std::string& value)
        {
            if(value == "scene3d") return RotationBodyMainViewMode::Scene3d;
            if(value == "section") return RotationBodyMainViewMode::Section;
            throw std::runtime_error("Unknown rotation-body main view mode.");
        }

        const char* rightWorkflowName(RotationBodyRightWorkflow value)
        {
            switch(value) {
            case RotationBodyRightWorkflow::ABBTranslation: return "abbTranslation";
            case RotationBodyRightWorkflow::WorkpieceCalibration:
                return "workpieceCalibration";
            case RotationBodyRightWorkflow::TrajectoryPlanning: return "trajectoryPlanning";
            }
            return "trajectoryPlanning";
        }

        RotationBodyRightWorkflow rightWorkflow(const std::string& value)
        {
            if(value == "trajectoryPlanning") {
                return RotationBodyRightWorkflow::TrajectoryPlanning;
            }
            if(value == "abbTranslation") return RotationBodyRightWorkflow::ABBTranslation;
            if(value == "workpieceCalibration") {
                return RotationBodyRightWorkflow::WorkpieceCalibration;
            }
            throw std::runtime_error("Unknown rotation-body right workflow.");
        }

        Json uiStateJson(const RotationBodyUiState& state)
        {
            return {
                { "workflow", workflowName(state.workflow) },
                { "mainViewMode", mainViewModeName(state.mainViewMode) },
                { "rightWorkflow", rightWorkflowName(state.rightWorkflow) }
            };
        }

        RotationBodyUiState uiState(const Json& value)
        {
            RotationBodyUiState result;
            result.workflow = workflow(value.at("workflow").get<std::string>());
            result.mainViewMode = mainViewMode(value.at("mainViewMode").get<std::string>());
            result.rightWorkflow = rightWorkflow(value.at("rightWorkflow").get<std::string>());
            return result;
        }

        std::string serialize(const RotationBodyPlanningDraft& draft)
        {
            Json commands = Json::array();
            for(const domain::RegionOverrideCommand& item : draft.editCommands) {
                commands.push_back(commandJson(item));
            }
            Json root = {
                { "schemaVersion", draft.schemaVersion },
                { "objectId", draft.objectId },
                { "sourcePath", draft.sourcePath },
                { "sourceFingerprint", draft.sourceFingerprint },
                { "meshFingerprint", draft.meshFingerprint },
                { "objectType", planningObjectTypeName(draft.objectType) },
                { "originalRotationAxis", signedAxisName(draft.originalRotationAxis) },
                { "toothOutwardAxis", signedAxisName(draft.toothOutwardAxis) },
                { "motherMaximumDiameterMeters", draft.motherMaximumDiameterMeters },
                { "planningFromMesh", transformJson(draft.planningFromMesh) },
                { "baseFromPlanning", transformJson(draft.baseFromPlanning) },
                { "automaticBaseline", transformJson(draft.automaticBaseline) },
                { "directedRotaryAxisInMesh", vector3Json(draft.directedRotaryAxisInMesh) },
                { "bottomAxisCenterInMesh", vector3Json(draft.bottomAxisCenterInMesh) },
                { "statistics", statisticsJson(draft.statistics) },
                { "stage", stageName(draft.stage) },
                { "parameters", parametersJson(draft.parameters) },
                { "publishFrame", publishFrameName(draft.publishFrame) },
                { "boundaryMode", boundaryModeName(draft.boundaryMode) },
                { "editCommands", std::move(commands) },
                { "appliedEditCommandCount", draft.appliedEditCommandCount },
                { "calibration", calibrationJson(draft.calibration) },
                { "uiState", uiStateJson(draft.uiState) }
            };
            if(draft.section) root["section"] = sectionJson(*draft.section);
            if(draft.automaticRegions) root["automaticRegions"] = regionsJson(*draft.automaticRegions);
            if(draft.boundary) root["boundary"] = boundaryJson(*draft.boundary);
            return root.dump();
        }

        RotationBodyPlanningDraft deserialize(const std::string& payload)
        {
            const Json root = Json::parse(payload);
            RotationBodyPlanningDraft draft;
            draft.schemaVersion = root.at("schemaVersion").get<int>();
            draft.objectId = root.at("objectId").get<std::string>();
            draft.sourcePath = root.at("sourcePath").get<std::string>();
            draft.sourceFingerprint = root.at("sourceFingerprint").get<std::string>();
            draft.meshFingerprint = root.at("meshFingerprint").get<std::uint64_t>();
            draft.objectType = planningObjectType(root.at("objectType").get<std::string>());
            draft.originalRotationAxis = signedAxis(root.at("originalRotationAxis").get<std::string>());
            draft.toothOutwardAxis = signedAxis(root.at("toothOutwardAxis").get<std::string>());
            draft.motherMaximumDiameterMeters = root.at("motherMaximumDiameterMeters").get<double>();
            draft.planningFromMesh = transform(root.at("planningFromMesh"));
            draft.baseFromPlanning = transform(root.at("baseFromPlanning"));
            draft.automaticBaseline = transform(root.at("automaticBaseline"));
            draft.directedRotaryAxisInMesh = vector3(root.at("directedRotaryAxisInMesh"));
            draft.bottomAxisCenterInMesh = vector3(root.at("bottomAxisCenterInMesh"));
            draft.statistics = statistics(root.at("statistics"));
            if(root.contains("calibration")) {
                draft.calibration = calibration(root.at("calibration"));
            } else if(draft.statistics.heightMeters > 0.0) {
                draft.calibration.workpieceHeightMeters = draft.statistics.heightMeters;
            }
            if(root.contains("uiState")) {
                draft.uiState = uiState(root.at("uiState"));
            }
            draft.stage = stage(root.at("stage").get<std::string>());
            draft.parameters = parameters(root.at("parameters"));
            draft.publishFrame = publishFrame(root.at("publishFrame").get<std::string>());
            draft.boundaryMode = boundaryMode(root.at("boundaryMode").get<std::string>());
            if(root.contains("section")) draft.section = section(root.at("section"));
            if(root.contains("automaticRegions")) {
                draft.automaticRegions = regions(root.at("automaticRegions"));
            }
            for(const Json& item : root.at("editCommands")) draft.editCommands.push_back(command(item));
            draft.appliedEditCommandCount = root.at("appliedEditCommandCount").get<std::size_t>();
            if(root.contains("boundary")) draft.boundary = boundary(root.at("boundary"));
            if(draft.objectId.empty() || draft.sourcePath.empty() || draft.sourceFingerprint.empty()) {
                throw std::runtime_error("Stored planning draft has an incomplete model identity.");
            }
            return draft;
        }

        RotationBodyPlanningDraft degradedDraft(
            RotationBodyPlanningDraft draft)
        {
            draft.stage = domain::PlanningStage::ModelLoaded;
            draft.section.reset();
            draft.automaticRegions.reset();
            draft.editCommands.clear();
            draft.appliedEditCommandCount = 0;
            draft.boundary.reset();
            draft.calibration = {};
            if(draft.statistics.heightMeters > 0.0) {
                draft.calibration.workpieceHeightMeters = draft.statistics.heightMeters;
            }
            return draft;
        }
    }

    bool RotationBodyPlanningDraftStore::write(
        simulation_project::ProjectDocument& document,
        const RotationBodyPlanningDraft& draft)
    {
        std::size_t matchingCount = 0;
        for(const simulation_project::ProjectExtensionDesc& extension : document.extensions) {
            if(extension.key != extensionKey) {
                continue;
            }
            ++matchingCount;
            if(extension.version > currentVersion) {
                throw std::runtime_error(
                    "A newer rotation-body planning draft exists and cannot be overwritten.");
            }
        }
        RotationBodyPlanningDraft versionedDraft = draft;
        versionedDraft.schemaVersion = currentVersion;
        const std::string payload = serialize(versionedDraft);
        if(matchingCount == 0) {
            simulation_project::ProjectExtensionDesc extension;
            extension.key = extensionKey;
            extension.version = currentVersion;
            extension.serializedPayload = payload;
            document.extensions.push_back(std::move(extension));
            return true;
        }

        bool changed = matchingCount != 1;
        bool keptFirst = false;
        std::vector<simulation_project::ProjectExtensionDesc> normalized;
        normalized.reserve(document.extensions.size() - matchingCount + 1);
        for(const simulation_project::ProjectExtensionDesc& extension : document.extensions) {
            if(extension.key != extensionKey) {
                normalized.push_back(extension);
                continue;
            }
            if(keptFirst) {
                continue;
            }
            keptFirst = true;
            simulation_project::ProjectExtensionDesc current = extension;
            if(current.version != currentVersion || current.serializedPayload != payload) {
                current.version = currentVersion;
                current.serializedPayload = payload;
                changed = true;
            }
            normalized.push_back(std::move(current));
        }
        if(changed) {
            document.extensions = std::move(normalized);
        }
        return changed;
    }

    DraftReadResult RotationBodyPlanningDraftStore::read(
        const simulation_project::ProjectDocument& document,
        const std::string& currentSourceFingerprint,
        std::uint64_t currentMeshFingerprint)
    {
        const simulation_project::ProjectExtensionDesc* found = nullptr;
        for(const simulation_project::ProjectExtensionDesc& extension : document.extensions) {
            if(extension.key != extensionKey) {
                continue;
            }
            if(found != nullptr) {
                return {
                    DraftReadStatus::InvalidPayload,
                    std::nullopt,
                    "The project contains duplicate rotation-body planning draft extensions."
                };
            }
            found = &extension;
        }
        if(found == nullptr) {
            return {};
        }
        if(found->version != currentVersion) {
            return {
                DraftReadStatus::UnsupportedVersion,
                std::nullopt,
                "The rotation-body planning draft uses an unsupported extension version."
            };
        }
        try {
            const Json payloadHeader = Json::parse(found->serializedPayload);
            const int payloadVersion = payloadHeader.at("schemaVersion").get<int>();
            if(payloadVersion != currentVersion) {
                return {
                    DraftReadStatus::UnsupportedVersion,
                    std::nullopt,
                    "The rotation-body planning draft uses an unsupported payload version."
                };
            }
            RotationBodyPlanningDraft draft = deserialize(found->serializedPayload);
            const bool sourceChanged =
                (!currentSourceFingerprint.empty() &&
                    currentSourceFingerprint != draft.sourceFingerprint) ||
                (currentMeshFingerprint != 0 && currentMeshFingerprint != draft.meshFingerprint);
            if(sourceChanged) {
                return {
                    DraftReadStatus::SourceChanged,
                    degradedDraft(std::move(draft)),
                    "The source model changed; section, regions and boundary were discarded."
                };
            }
            return { DraftReadStatus::Loaded, std::move(draft), {} };
        } catch(const std::exception& exception) {
            return { DraftReadStatus::InvalidPayload, std::nullopt, exception.what() };
        }
    }

    bool RotationBodyPlanningDraftStore::erase(simulation_project::ProjectDocument& document)
    {
        const auto previousSize = document.extensions.size();
        document.extensions.erase(
            std::remove_if(
                document.extensions.begin(),
                document.extensions.end(),
                [](const simulation_project::ProjectExtensionDesc& extension) {
                    return extension.key == extensionKey;
                }),
            document.extensions.end());
        return document.extensions.size() != previousSize;
    }
}
