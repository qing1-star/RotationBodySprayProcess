#include "RotationBodyTrajectoryProjectStore.h"

#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/TrajectoryGroupEditor.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

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
            Eigen::Vector3d result(
                value.at(0).get<double>(),
                value.at(1).get<double>(),
                value.at(2).get<double>());
            if(!result.allFinite()) {
                throw std::runtime_error("Vector contains a non-finite value.");
            }
            return result;
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

        const char* boundaryModeName(domain::BoundaryMode value)
        {
            return value == domain::BoundaryMode::ToothTopEnvelope
                ? "toothTopEnvelope"
                : "maximumToothTopY";
        }

        domain::BoundaryMode boundaryMode(const std::string& value)
        {
            if(value == "maximumToothTopY") {
                return domain::BoundaryMode::MaximumToothTopY;
            }
            if(value == "toothTopEnvelope") {
                return domain::BoundaryMode::ToothTopEnvelope;
            }
            throw std::runtime_error("Unknown boundary mode.");
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

        Json sectionJson(const domain::SectionContour& section)
        {
            Json pointsYz = Json::array();
            for(const Eigen::Vector2d& point : section.pointsYz) {
                pointsYz.push_back(vector2Json(point));
            }
            Json points3d = Json::array();
            for(const Eigen::Vector3d& point : section.points3d) {
                points3d.push_back(vector3Json(point));
            }
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
            for(const Json& point : value.at("pointsYz")) {
                result.pointsYz.push_back(vector2(point));
            }
            for(const Json& point : value.at("points3d")) {
                result.points3d.push_back(vector3(point));
            }
            result.cumulativeArcLength = value.at("cumulativeArcLength")
                .get<std::vector<double>>();
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
            for(const domain::RegionLabel label : regions.segmentLabels) {
                labels.push_back(regionLabelName(label));
            }
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
            result.segmentConfidence = value.value(
                "confidence", std::vector<double>{});
            result.diagnostics = value.value("diagnostics", std::vector<std::string>{});
            if(!result.segmentConfidence.empty() &&
                result.segmentLabels.size() != result.segmentConfidence.size()) {
                throw std::runtime_error("Stored region dimensions are invalid.");
            }
            return result;
        }

        Json boundaryJson(const domain::SprayBoundary& boundary)
        {
            Json polygon = Json::array();
            for(const Eigen::Vector2d& point : boundary.polygonYz) {
                polygon.push_back(vector2Json(point));
            }
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
            for(const Json& point : value.at("polygonYz")) {
                result.polygonYz.push_back(vector2(point));
            }
            result.minimumY = value.at("minimumY").get<double>();
            result.maximumY = value.at("maximumY").get<double>();
            result.minimumZ = value.at("minimumZ").get<double>();
            result.maximumZ = value.at("maximumZ").get<double>();
            result.outerLineSlopeYPerZ = value.at("outerLineSlopeYPerZ").get<double>();
            result.outerLineInterceptY = value.at("outerLineInterceptY").get<double>();
            return result;
        }

        Json parametersJson(const domain::TrajectoryGenerationParameters& parameters)
        {
            return {
                { "sprayDistanceMeters", parameters.sprayDistanceMeters },
                { "tiltRadians", parameters.tiltRadians },
                { "speedMetersPerSecond", parameters.speedMetersPerSecond },
                { "startExtensionMeters", parameters.startExtensionMeters },
                { "endExtensionMeters", parameters.endExtensionMeters },
                { "pointCount", parameters.pointCount },
                { "positionerRpm", parameters.positionerRpm },
                { "reversed", parameters.reversed }
            };
        }

        domain::TrajectoryGenerationParameters parameters(const Json& value)
        {
            domain::TrajectoryGenerationParameters result;
            result.sprayDistanceMeters = value.at("sprayDistanceMeters").get<double>();
            result.tiltRadians = value.at("tiltRadians").get<double>();
            result.speedMetersPerSecond = value.at("speedMetersPerSecond").get<double>();
            result.startExtensionMeters = value.at("startExtensionMeters").get<double>();
            result.endExtensionMeters = value.at("endExtensionMeters").get<double>();
            result.pointCount = value.at("pointCount").get<std::size_t>();
            result.positionerRpm = value.at("positionerRpm").get<double>();
            result.reversed = value.at("reversed").get<bool>();
            return result;
        }

        Json pointJson(const domain::TrajectoryPosePoint& point)
        {
            return {
                { "timeSeconds", point.timeSeconds },
                { "planningFromTool", transformJson(point.planningFromTool) },
                { "interpolated", point.interpolated }
            };
        }

        domain::TrajectoryPosePoint point(const Json& value)
        {
            domain::TrajectoryPosePoint result;
            result.timeSeconds = value.at("timeSeconds").get<double>();
            result.planningFromTool = transform(value.at("planningFromTool"));
            result.interpolated = value.at("interpolated").get<bool>();
            return result;
        }

        Json pointsJson(const std::vector<domain::TrajectoryPosePoint>& points)
        {
            Json result = Json::array();
            for(const domain::TrajectoryPosePoint& value : points) {
                result.push_back(pointJson(value));
            }
            return result;
        }

        std::vector<domain::TrajectoryPosePoint> points(const Json& value)
        {
            std::vector<domain::TrajectoryPosePoint> result;
            result.reserve(value.size());
            for(const Json& item : value) {
                result.push_back(point(item));
            }
            return result;
        }

        Json trajectoryJson(const domain::PlannedTrajectory& trajectory)
        {
            return {
                { "parameters", parametersJson(trajectory.parameters) },
                { "sourceBoundary", boundaryJson(trajectory.sourceBoundary) },
                { "targetSurfaceStart", vector3Json(trajectory.targetSurfaceStart) },
                { "targetSurfaceEnd", vector3Json(trajectory.targetSurfaceEnd) },
                { "linearPoints", pointsJson(trajectory.linearPoints) },
                { "relativeHelicalPoints", pointsJson(trajectory.relativeHelicalPoints) },
                { "metrics", {
                    { "pathLengthMeters", trajectory.metrics.pathLengthMeters },
                    { "durationSeconds", trajectory.metrics.durationSeconds },
                    { "minimumPointIntervalSeconds", trajectory.metrics.minimumPointIntervalSeconds },
                    { "maximumPointIntervalSeconds", trajectory.metrics.maximumPointIntervalSeconds }
                } }
            };
        }

        domain::PlannedTrajectory trajectory(const Json& value)
        {
            domain::PlannedTrajectory result;
            result.parameters = parameters(value.at("parameters"));
            result.sourceBoundary = boundary(value.at("sourceBoundary"));
            result.targetSurfaceStart = vector3(value.at("targetSurfaceStart"));
            result.targetSurfaceEnd = vector3(value.at("targetSurfaceEnd"));
            result.linearPoints = points(value.at("linearPoints"));
            result.relativeHelicalPoints = points(value.at("relativeHelicalPoints"));
            const Json& metrics = value.at("metrics");
            result.metrics.pathLengthMeters = metrics.at("pathLengthMeters").get<double>();
            result.metrics.durationSeconds = metrics.at("durationSeconds").get<double>();
            result.metrics.minimumPointIntervalSeconds =
                metrics.at("minimumPointIntervalSeconds").get<double>();
            result.metrics.maximumPointIntervalSeconds =
                metrics.at("maximumPointIntervalSeconds").get<double>();
            return result;
        }

        Json groupJson(const domain::TrajectoryGroup& group)
        {
            Json passes = Json::array();
            for(const domain::TrajectoryPass& pass : group.passes) {
                passes.push_back({
                    { "id", pass.id },
                    { "order", pass.order },
                    { "visible", pass.visible },
                    { "startOffsetSeconds", pass.startOffsetSeconds },
                    { "transitionAfterSeconds", pass.transitionAfterSeconds },
                    { "trajectory", trajectoryJson(pass.trajectory) }
                });
            }
            return { { "passes", std::move(passes) }, { "cycleCount", group.cycleCount } };
        }

        domain::TrajectoryGroup group(const Json& value)
        {
            domain::TrajectoryGroup result;
            result.cycleCount = value.value("cycleCount", std::size_t{ 1 });
            if(result.cycleCount < 1 || result.cycleCount > 100) {
                throw std::runtime_error("Trajectory group cycle count must be between 1 and 100.");
            }
            for(const Json& item : value.at("passes")) {
                domain::TrajectoryPass pass;
                pass.id = item.at("id").get<std::string>();
                pass.order = item.at("order").get<int>();
                pass.visible = item.at("visible").get<bool>();
                pass.startOffsetSeconds = item.at("startOffsetSeconds").get<double>();
                pass.transitionAfterSeconds = item.at("transitionAfterSeconds").get<double>();
                pass.trajectory = trajectory(item.at("trajectory"));
                result.passes.push_back(std::move(pass));
            }
            return result;
        }

        Json workspaceJson(const domain::TrajectoryWorkspace& workspace)
        {
            Json result = {
                { "parameters", parametersJson(workspace.parameters) },
                { "editingPassId", workspace.editingPassId },
                { "displayMode", workspace.displayMode ==
                    domain::TrajectoryDisplayMode::RelativeHelical
                    ? "relativeHelical"
                    : "linear" },
                { "group", groupJson(workspace.group) }
            };
            if(workspace.currentTrajectory) {
                result["currentTrajectory"] = trajectoryJson(*workspace.currentTrajectory);
            }
            return result;
        }

        domain::TrajectoryWorkspace workspace(const Json& value)
        {
            domain::TrajectoryWorkspace result;
            result.parameters = parameters(value.at("parameters"));
            result.editingPassId = value.at("editingPassId").get<std::string>();
            if(value.contains("displayMode")) {
                const std::string mode = value.at("displayMode").get<std::string>();
                if(mode == "relativeHelical") {
                    result.displayMode = domain::TrajectoryDisplayMode::RelativeHelical;
                } else if(mode != "linear") {
                    throw std::runtime_error("Unknown trajectory display mode.");
                }
            }
            result.group = group(value.at("group"));
            if(value.contains("currentTrajectory")) {
                result.currentTrajectory = trajectory(value.at("currentTrajectory"));
            }
            return result;
        }

        Json rapidSettingsJson(const domain::RapidExportSettings& settings)
        {
            return {
                { "safetyPositionBaseMeters", vector3Json(settings.safetyPositionBaseMeters) },
                { "safetySpeedMetersPerSecond", settings.safetySpeedMetersPerSecond },
                { "moduleName", settings.moduleName },
                { "fileName", settings.fileName },
                { "toolDataName", settings.toolDataName },
                { "outputDirectory", settings.outputDirectory }
            };
        }

        domain::RapidExportSettings rapidSettings(const Json& value)
        {
            domain::RapidExportSettings result;
            result.safetyPositionBaseMeters = vector3(value.at("safetyPositionBaseMeters"));
            result.safetySpeedMetersPerSecond =
                value.at("safetySpeedMetersPerSecond").get<double>();
            result.moduleName = value.at("moduleName").get<std::string>();
            result.fileName = value.at("fileName").get<std::string>();
            result.toolDataName = value.at("toolDataName").get<std::string>();
            result.outputDirectory = value.at("outputDirectory").get<std::string>();
            return result;
        }

        Json sequenceJson(const std::vector<domain::RapidSequenceEntry>& sequence)
        {
            Json result = Json::array();
            for(const domain::RapidSequenceEntry& entry : sequence) {
                result.push_back({
                    { "kind", entry.kind == domain::RapidSequenceEntryKind::Trajectory
                        ? "trajectory"
                        : "safetyPoint" },
                    { "trajectoryPassId", entry.trajectoryPassId }
                });
            }
            return result;
        }

        std::vector<domain::RapidSequenceEntry> sequence(const Json& value)
        {
            std::vector<domain::RapidSequenceEntry> result;
            for(const Json& item : value) {
                const std::string kind = item.at("kind").get<std::string>();
                domain::RapidSequenceEntry entry;
                if(kind == "trajectory") {
                    entry.kind = domain::RapidSequenceEntryKind::Trajectory;
                } else if(kind == "safetyPoint") {
                    entry.kind = domain::RapidSequenceEntryKind::SafetyPoint;
                } else {
                    throw std::runtime_error("Unknown ABB sequence entry kind.");
                }
                entry.trajectoryPassId = item.at("trajectoryPassId").get<std::string>();
                result.push_back(std::move(entry));
            }
            return result;
        }

        bool upsert(
            simulation_project::ProjectDocument& document,
            const char* key,
            int version,
            std::string payload)
        {
            std::size_t matchingCount = 0;
            for(const simulation_project::ProjectExtensionDesc& extension : document.extensions) {
                if(extension.key == key) {
                    ++matchingCount;
                    if(extension.version > version) {
                        throw std::runtime_error(
                            "A newer rotation-body trajectory extension cannot be overwritten.");
                    }
                }
            }
            if(matchingCount == 0) {
                document.extensions.push_back({
                    key,
                    version,
                    std::move(payload)
                });
                return true;
            }

            bool changed = matchingCount != 1;
            bool kept = false;
            std::vector<simulation_project::ProjectExtensionDesc> normalized;
            normalized.reserve(document.extensions.size() - matchingCount + 1);
            for(const simulation_project::ProjectExtensionDesc& extension : document.extensions) {
                if(extension.key != key) {
                    normalized.push_back(extension);
                    continue;
                }
                if(kept) {
                    continue;
                }
                kept = true;
                simulation_project::ProjectExtensionDesc updated = extension;
                if(updated.version != version ||
                    updated.serializedPayload != payload) {
                    updated.version = version;
                    updated.serializedPayload = payload;
                    changed = true;
                }
                normalized.push_back(std::move(updated));
            }
            if(changed) {
                document.extensions = std::move(normalized);
            }
            return changed;
        }

        bool erase(simulation_project::ProjectDocument& document, const char* key)
        {
            const std::size_t previousSize = document.extensions.size();
            document.extensions.erase(
                std::remove_if(
                    document.extensions.begin(),
                    document.extensions.end(),
                    [key](const simulation_project::ProjectExtensionDesc& extension) {
                        return extension.key == key;
                    }),
                document.extensions.end());
            return document.extensions.size() != previousSize;
        }

        const simulation_project::ProjectExtensionDesc* findUnique(
            const simulation_project::ProjectDocument& document,
            const char* key,
            bool& duplicate)
        {
            duplicate = false;
            const simulation_project::ProjectExtensionDesc* found = nullptr;
            for(const simulation_project::ProjectExtensionDesc& extension : document.extensions) {
                if(extension.key != key) {
                    continue;
                }
                if(found != nullptr) {
                    duplicate = true;
                    return nullptr;
                }
                found = &extension;
            }
            return found;
        }
    }

    bool RotationBodyTrajectoryProjectStore::writeWorkspace(
        simulation_project::ProjectDocument& document,
        const RotationBodyTrajectoryDraft& draft)
    {
        RotationBodyTrajectoryDraft versioned = draft;
        versioned.schemaVersion = workspaceVersion;
        Json root = {
            { "schemaVersion", versioned.schemaVersion },
            { "objectId", versioned.objectId },
            { "sourceFingerprint", versioned.sourceFingerprint },
            { "meshFingerprint", versioned.meshFingerprint },
            { "workspace", workspaceJson(versioned.workspace) },
            { "rapidSettings", rapidSettingsJson(versioned.rapidSettings) },
            { "rapidSequence", sequenceJson(versioned.rapidSequence) }
        };
        return upsert(document, workspaceExtensionKey, workspaceVersion, root.dump());
    }

    TrajectoryWorkspaceReadResult RotationBodyTrajectoryProjectStore::readWorkspace(
        const simulation_project::ProjectDocument& document)
    {
        bool duplicate = false;
        const simulation_project::ProjectExtensionDesc* extension =
            findUnique(document, workspaceExtensionKey, duplicate);
        if(duplicate) {
            return {
                TrajectoryProjectReadStatus::InvalidPayload,
                std::nullopt,
                "The project contains duplicate rotation-body trajectory workspace extensions."
            };
        }
        if(extension == nullptr) {
            return {};
        }
        if(extension->version != workspaceVersion) {
            return {
                TrajectoryProjectReadStatus::UnsupportedVersion,
                std::nullopt,
                "The trajectory workspace uses an unsupported extension version."
            };
        }
        try {
            const Json root = Json::parse(extension->serializedPayload);
            if(root.at("schemaVersion").get<int>() != workspaceVersion) {
                return {
                    TrajectoryProjectReadStatus::UnsupportedVersion,
                    std::nullopt,
                    "The trajectory workspace uses an unsupported payload version."
                };
            }
            RotationBodyTrajectoryDraft draft;
            draft.schemaVersion = workspaceVersion;
            draft.objectId = root.at("objectId").get<std::string>();
            draft.sourceFingerprint = root.at("sourceFingerprint").get<std::string>();
            draft.meshFingerprint = root.at("meshFingerprint").get<std::uint64_t>();
            draft.workspace = workspace(root.at("workspace"));
            draft.rapidSettings = rapidSettings(root.at("rapidSettings"));
            draft.rapidSequence = sequence(root.at("rapidSequence"));
            return {
                TrajectoryProjectReadStatus::Loaded,
                std::move(draft),
                {}
            };
        } catch(const std::exception& exception) {
            return {
                TrajectoryProjectReadStatus::InvalidPayload,
                std::nullopt,
                exception.what()
            };
        }
    }

    bool RotationBodyTrajectoryProjectStore::eraseWorkspace(
        simulation_project::ProjectDocument& document)
    {
        return erase(document, workspaceExtensionKey);
    }

    bool RotationBodyTrajectoryProjectStore::writePublishedPlan(
        simulation_project::ProjectDocument& document,
        const domain::PublishedTrajectoryPlan& plan)
    {
        domain::PublishedTrajectoryPlan versioned = plan;
        versioned.schemaVersion = currentVersion;
        Json root = {
            { "schemaVersion", versioned.schemaVersion },
            { "objectId", versioned.objectId },
            { "baseFromPlanning", transformJson(versioned.baseFromPlanning) },
            { "planningFromMesh", transformJson(versioned.planningFromMesh) },
            { "group", groupJson(versioned.group) },
            { "safetyPositionBaseMeters", vector3Json(versioned.safetyPositionBaseMeters) },
            { "safetySpeedMetersPerSecond", versioned.safetySpeedMetersPerSecond },
            { "executionSequence", sequenceJson(versioned.executionSequence) }
        };
        if(versioned.section) {
            root["section"] = sectionJson(*versioned.section);
        }
        if(versioned.regions) {
            if(!versioned.section || !versioned.regions->matches(*versioned.section)) {
                throw std::runtime_error(
                    "Published trajectory region data does not match its section contour.");
            }
            root["regions"] = regionsJson(*versioned.regions);
        }
        return upsert(document, publishedExtensionKey, currentVersion, root.dump());
    }

    PublishedTrajectoryReadResult RotationBodyTrajectoryProjectStore::readPublishedPlan(
        const simulation_project::ProjectDocument& document)
    {
        bool duplicate = false;
        const simulation_project::ProjectExtensionDesc* extension =
            findUnique(document, publishedExtensionKey, duplicate);
        if(duplicate) {
            return {
                TrajectoryProjectReadStatus::InvalidPayload,
                std::nullopt,
                "The project contains duplicate published trajectory extensions."
            };
        }
        if(extension == nullptr) {
            return {};
        }
        if(extension->version <
                smrobot::spray::rotationbody::kPublishedTrajectoryPlanMinimumSchemaVersion ||
            extension->version > currentVersion) {
            return {
                TrajectoryProjectReadStatus::UnsupportedVersion,
                std::nullopt,
                "The published trajectory plan uses an unsupported extension version."
            };
        }
        try {
            const Json root = Json::parse(extension->serializedPayload);
            const int schemaVersion = root.at("schemaVersion").get<int>();
            if(schemaVersion != extension->version ||
                schemaVersion <
                    smrobot::spray::rotationbody::kPublishedTrajectoryPlanMinimumSchemaVersion ||
                schemaVersion > currentVersion) {
                return {
                    TrajectoryProjectReadStatus::UnsupportedVersion,
                    std::nullopt,
                    "The published trajectory plan uses an unsupported payload version."
                };
            }
            domain::PublishedTrajectoryPlan plan;
            plan.schemaVersion = schemaVersion;
            plan.objectId = root.at("objectId").get<std::string>();
            plan.baseFromPlanning = transform(root.at("baseFromPlanning"));
            if(root.contains("planningFromMesh")) {
                plan.planningFromMesh = transform(root.at("planningFromMesh"));
            }
            if(root.contains("section")) {
                plan.section = section(root.at("section"));
            }
            if(root.contains("regions")) {
                plan.regions = regions(root.at("regions"));
                if(!plan.section || !plan.regions->matches(*plan.section)) {
                    throw std::runtime_error(
                        "Published trajectory regions do not match the section contour.");
                }
            }
            plan.group = group(root.at("group"));
            if(root.contains("safetyPositionBaseMeters")) {
                plan.safetyPositionBaseMeters = vector3(root.at("safetyPositionBaseMeters"));
            }
            if(root.contains("safetySpeedMetersPerSecond")) {
                plan.safetySpeedMetersPerSecond =
                    root.at("safetySpeedMetersPerSecond").get<double>();
            }
            if(root.contains("executionSequence")) {
                plan.executionSequence = sequence(root.at("executionSequence"));
            } else {
                plan.executionSequence.push_back({ domain::RapidSequenceEntryKind::SafetyPoint, {} });
                for(const domain::TrajectoryPass& pass : plan.group.passes) {
                    plan.executionSequence.push_back({ domain::RapidSequenceEntryKind::Trajectory, pass.id });
                }
                plan.executionSequence.push_back({ domain::RapidSequenceEntryKind::SafetyPoint, {} });
            }
            if(plan.objectId.empty()) {
                throw std::runtime_error("Published trajectory workpiece id is empty.");
            }
            const domain::PlanningResult<void> validation =
                domain::TrajectoryGroupEditor::validate(plan.group);
            if(!validation) {
                throw std::runtime_error(validation.error.message);
            }
            return {
                TrajectoryProjectReadStatus::Loaded,
                std::move(plan),
                {}
            };
        } catch(const std::exception& exception) {
            return {
                TrajectoryProjectReadStatus::InvalidPayload,
                std::nullopt,
                exception.what()
            };
        }
    }

    bool RotationBodyTrajectoryProjectStore::erasePublishedPlan(
        simulation_project::ProjectDocument& document)
    {
        return erase(document, publishedExtensionKey);
    }

    bool RotationBodyTrajectoryProjectStore::eraseAll(
        simulation_project::ProjectDocument& document)
    {
        const bool workspaceErased = eraseWorkspace(document);
        return erasePublishedPlan(document) || workspaceErased;
    }
}
