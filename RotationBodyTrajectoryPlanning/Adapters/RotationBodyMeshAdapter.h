#pragma once

#include <RotationBodyTrajectoryPlanning/Core/PlanningTypes.h>
#include <RotationBodyTrajectoryPlanning/Core/TriangleMesh.h>

#include <filesystem>
#include <string>

namespace assetcore
{
    class ModelDesc;
}

namespace simulation_project
{
    class ProjectSession;
    struct SceneObjectDesc;
}

namespace smrobot::workbench::spray::rotationbody
{
    namespace domain = smrobot::spray::rotationbody;

    struct RotationBodyMeshLoad
    {
        domain::TriangleMesh mesh;
        std::filesystem::path resolvedSourcePath;
        std::string sourceFingerprint;
    };

    class RotationBodyMeshAdapter
    {
    public:
        static domain::PlanningResult<RotationBodyMeshLoad> load(
            const simulation_project::ProjectSession& session,
            const simulation_project::SceneObjectDesc& object);
        static domain::PlanningResult<std::filesystem::path> resolveSourcePath(
            const simulation_project::ProjectSession& session,
            const simulation_project::SceneObjectDesc& object);
        static domain::PlanningResult<RotationBodyMeshLoad> loadResolvedSource(
            const std::filesystem::path& sourcePath,
            double visualScale);
        static domain::PlanningResult<domain::TriangleMesh> build(
            const assetcore::ModelDesc& model);
        static std::string makeSourceFingerprint(
            const std::filesystem::path& resolvedSourcePath,
            const domain::TriangleMesh& mesh);
    };
}
