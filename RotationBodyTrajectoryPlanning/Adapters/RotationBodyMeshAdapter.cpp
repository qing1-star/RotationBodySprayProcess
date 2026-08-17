#include "RotationBodyMeshAdapter.h"

#include <AssetCore/AssetManager.h>
#include <AssetCore/ModelDesc.h>
#include <SimulationProject/AssetResolver.h>
#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectSession.h>
#include <SimulationProject/RuntimePaths.h>

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <system_error>
#include <unordered_map>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        std::filesystem::path projectBasePath(const simulation_project::ProjectSession& session)
        {
            return session.path().empty()
                ? simulation_project::RuntimePaths::applicationRoot()
                : session.path().parent_path();
        }

        simulation_project::AssetResolveContext makeResolveContext(
            const simulation_project::ProjectSession& session)
        {
            simulation_project::AssetResolveContext context;
            context.projectBasePath = projectBasePath(session);
            context.sourceRootPath = simulation_project::RuntimePaths::sourceRoot();
            context.dataRootPath = simulation_project::RuntimePaths::dataRoot();
            context.appRootPath = simulation_project::RuntimePaths::applicationRoot();
            context.generatedAssetRootPath =
                simulation_project::AssetResolver::appGeneratedAssetRoot(context);
            context.assetSearchPaths = session.document().assetSearchPaths;
            return context;
        }

        bool finiteLocalTransform(const glm::mat4& transform)
        {
            for(int column = 0; column < 4; ++column) {
                for(int row = 0; row < 4; ++row) {
                    if(!std::isfinite(transform[column][row])) {
                        return false;
                    }
                }
            }
            return true;
        }

        domain::PlanningResult<Eigen::Vector3d> transformedPosition(
            const glm::mat4& localTransform,
            const Eigen::Vector3f& position)
        {
            if(!position.allFinite()) {
                return domain::PlanningResult<Eigen::Vector3d>::failure(
                    domain::PlanningErrorCode::NonFiniteGeometry,
                    "A submesh contains a non-finite position.");
            }
            const glm::vec4 corrected = localTransform * glm::vec4(
                position.x(),
                position.y(),
                position.z(),
                1.0f);
            if(!std::isfinite(corrected.x) || !std::isfinite(corrected.y) ||
                !std::isfinite(corrected.z) || !std::isfinite(corrected.w) ||
                std::abs(corrected.w - 1.0f) > 1.0e-5f) {
                return domain::PlanningResult<Eigen::Vector3d>::failure(
                    domain::PlanningErrorCode::NonFiniteGeometry,
                    "The model local transform produced an invalid homogeneous position.");
            }
            return domain::PlanningResult<Eigen::Vector3d>::success(Eigen::Vector3d(
                static_cast<double>(corrected.x),
                static_cast<double>(corrected.y),
                static_cast<double>(corrected.z)
            ));
        }

        domain::PlanningResult<int> appendPosition(
            domain::TriangleMesh& mesh,
            const glm::mat4& localTransform,
            const Eigen::Vector3f& position)
        {
            if(mesh.positions.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                return domain::PlanningResult<int>::failure(
                    domain::PlanningErrorCode::InvalidTriangleIndex,
                    "The workpiece has too many referenced vertices for the planning mesh.");
            }
            domain::PlanningResult<Eigen::Vector3d> converted =
                transformedPosition(localTransform, position);
            if(!converted) {
                return domain::PlanningResult<int>::failure(
                    converted.error.code,
                    converted.error.message);
            }
            const int index = static_cast<int>(mesh.positions.size());
            mesh.positions.push_back(converted.value);
            return domain::PlanningResult<int>::success(index);
        }
    }

    domain::PlanningResult<RotationBodyMeshLoad> RotationBodyMeshAdapter::load(
        const simulation_project::ProjectSession& session,
        const simulation_project::SceneObjectDesc& object)
    {
        if(!std::isfinite(object.visualScale) || object.visualScale <= 0.0) {
            return domain::PlanningResult<RotationBodyMeshLoad>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The workpiece visual scale must be finite and positive.");
        }

        domain::PlanningResult<std::filesystem::path> sourcePath =
            resolveSourcePath(session, object);
        if(!sourcePath) {
            return domain::PlanningResult<RotationBodyMeshLoad>::failure(
                sourcePath.error.code,
                sourcePath.error.message);
        }
        return loadResolvedSource(sourcePath.value, object.visualScale);
    }

    domain::PlanningResult<std::filesystem::path>
    RotationBodyMeshAdapter::resolveSourcePath(
        const simulation_project::ProjectSession& session,
        const simulation_project::SceneObjectDesc& object)
    {
        if(object.sourcePath.empty()) {
            return domain::PlanningResult<std::filesystem::path>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The workpiece does not reference a source model.");
        }
        const std::filesystem::path sourcePath =
            simulation_project::AssetResolver::resolveProjectPath(
                makeResolveContext(session),
                object.sourcePath);
        if(sourcePath.empty()) {
            return domain::PlanningResult<std::filesystem::path>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "The workpiece source model path could not be resolved.");
        }
        return domain::PlanningResult<std::filesystem::path>::success(sourcePath);
    }

    domain::PlanningResult<RotationBodyMeshLoad>
    RotationBodyMeshAdapter::loadResolvedSource(
        const std::filesystem::path& sourcePath,
        double visualScale)
    {
        if(sourcePath.empty() || !std::isfinite(visualScale) || visualScale <= 0.0) {
            return domain::PlanningResult<RotationBodyMeshLoad>::failure(
                domain::PlanningErrorCode::InvalidArgument,
                "A resolved source path and positive visual scale are required.");
        }
        std::string loadError;
        const std::shared_ptr<assetcore::ModelDesc> model =
            assetcore::AssetManager::instance().tryLoadModel(
                sourcePath.generic_u8string(),
                static_cast<float>(visualScale),
                &loadError);
        if(!model) {
            return domain::PlanningResult<RotationBodyMeshLoad>::failure(
                domain::PlanningErrorCode::EmptyMesh,
                loadError.empty() ? "Failed to load the workpiece mesh." : loadError);
        }

        domain::PlanningResult<domain::TriangleMesh> converted = build(*model);
        if(!converted) {
            return domain::PlanningResult<RotationBodyMeshLoad>::failure(
                converted.error.code,
                converted.error.message);
        }
        RotationBodyMeshLoad result;
        result.mesh = std::move(converted.value);
        result.resolvedSourcePath = sourcePath;
        result.sourceFingerprint = makeSourceFingerprint(sourcePath, result.mesh);
        return domain::PlanningResult<RotationBodyMeshLoad>::success(std::move(result));
    }

    domain::PlanningResult<domain::TriangleMesh> RotationBodyMeshAdapter::build(
        const assetcore::ModelDesc& model)
    {
        domain::TriangleMesh mesh;
        // AssetCore currently exposes only a non-const local-transform getter.
        // Runtime rendering reads the same matrix without mutating it.
        const glm::mat4 localTransform = const_cast<assetcore::ModelDesc&>(model).get_local();
        if(!finiteLocalTransform(localTransform)) {
            return domain::PlanningResult<domain::TriangleMesh>::failure(
                domain::PlanningErrorCode::NonFiniteGeometry,
                "The model local transform contains a non-finite value.");
        }
        for(const assetcore::SubMeshDesc& subMesh : model.subMeshes()) {
            const assetcore::GeometryDesc& geometry = subMesh.geometry;
            if(geometry.positions.empty()) {
                if(!geometry.indices.empty()) {
                    return domain::PlanningResult<domain::TriangleMesh>::failure(
                        domain::PlanningErrorCode::InvalidTriangleIndex,
                        "A submesh has indices but no positions.");
                }
                continue;
            }

            if(geometry.indices.empty()) {
                if(geometry.positions.size() % 3 != 0) {
                    return domain::PlanningResult<domain::TriangleMesh>::failure(
                        domain::PlanningErrorCode::InvalidTriangleIndex,
                        "A non-indexed submesh must contain complete triangle triplets.");
                }
                for(std::size_t index = 0; index < geometry.positions.size(); index += 3) {
                    Eigen::Vector3i triangle;
                    for(int corner = 0; corner < 3; ++corner) {
                        domain::PlanningResult<int> appended = appendPosition(
                            mesh,
                            localTransform,
                            geometry.positions[index + static_cast<std::size_t>(corner)]);
                        if(!appended) {
                            return domain::PlanningResult<domain::TriangleMesh>::failure(
                                appended.error.code,
                                appended.error.message);
                        }
                        triangle[corner] = appended.value;
                    }
                    mesh.triangles.push_back(triangle);
                }
                continue;
            }

            if(geometry.indices.size() % 3 != 0) {
                return domain::PlanningResult<domain::TriangleMesh>::failure(
                    domain::PlanningErrorCode::InvalidTriangleIndex,
                    "An indexed submesh must contain complete triangle triplets.");
            }
            std::unordered_map<std::uint32_t, int> mappedIndices;
            mappedIndices.reserve(std::min(geometry.positions.size(), geometry.indices.size()));
            for(std::size_t index = 0; index < geometry.indices.size(); index += 3) {
                Eigen::Vector3i triangle;
                for(int corner = 0; corner < 3; ++corner) {
                    const std::uint32_t sourceIndex = geometry.indices[index + corner];
                    if(sourceIndex >= geometry.positions.size()) {
                        return domain::PlanningResult<domain::TriangleMesh>::failure(
                            domain::PlanningErrorCode::InvalidTriangleIndex,
                            "A submesh triangle references an out-of-range position.");
                    }
                    const auto found = mappedIndices.find(sourceIndex);
                    if(found != mappedIndices.end()) {
                        triangle[corner] = found->second;
                        continue;
                    }
                    domain::PlanningResult<int> appended = appendPosition(
                        mesh,
                        localTransform,
                        geometry.positions[sourceIndex]);
                    if(!appended) {
                        return domain::PlanningResult<domain::TriangleMesh>::failure(
                            appended.error.code,
                            appended.error.message);
                    }
                    mappedIndices.emplace(sourceIndex, appended.value);
                    triangle[corner] = appended.value;
                }
                mesh.triangles.push_back(triangle);
            }
        }

        const domain::PlanningResult<void> validation = mesh.validate();
        if(!validation) {
            return domain::PlanningResult<domain::TriangleMesh>::failure(
                validation.error.code,
                validation.error.message);
        }
        return domain::PlanningResult<domain::TriangleMesh>::success(std::move(mesh));
    }

    std::string RotationBodyMeshAdapter::makeSourceFingerprint(
        const std::filesystem::path& resolvedSourcePath,
        const domain::TriangleMesh& mesh)
    {
        std::error_code error;
        const std::uintmax_t fileSize = std::filesystem::file_size(resolvedSourcePath, error);
        const std::uintmax_t safeFileSize = error ? 0 : fileSize;
        error.clear();
        const auto writeTime = std::filesystem::last_write_time(resolvedSourcePath, error);
        const long long writeTicks = error
            ? 0
            : static_cast<long long>(writeTime.time_since_epoch().count());

        std::ostringstream fingerprint;
        fingerprint << "v1:" << safeFileSize << ':' << writeTicks << ':'
            << std::hex << std::setfill('0') << std::setw(16) << mesh.stableFingerprint();
        return fingerprint.str();
    }
}
