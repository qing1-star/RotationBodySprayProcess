#include "RotationBodyPlanningDocumentOperations.h"

#include "RotationBodyPlanningDraftStore.h"
#include "RotationBodyTrajectoryProjectStore.h"

#include <SimulationProject/ProjectDocumentService.h>

namespace smrobot::workbench::spray::rotationbody
{
    PlanningObjectReplacementResult
    RotationBodyPlanningDocumentOperations::replacePlanningObject(
        simulation_project::ProjectDocumentService& service,
        const std::string& previousObjectId,
        const std::string& currentObjectId)
    {
        if(currentObjectId.empty() || service.findSceneObject(currentObjectId) == nullptr) {
            return { false, false, "The newly imported planning workpiece does not exist." };
        }
        if(previousObjectId.empty() || previousObjectId == currentObjectId) {
            return { true, false, {} };
        }
        if(service.findSceneObject(previousObjectId) == nullptr) {
            return { false, false, "The previous planning workpiece does not exist." };
        }

        simulation_project::ProjectReferenceCleanupReport cleanup;
        std::string error;
        if(!service.removeObject(previousObjectId, cleanup, &error)) {
            return {
                false,
                false,
                error.empty() ? "Failed to remove the previous planning workpiece." : error
            };
        }
        RotationBodyPlanningDraftStore::erase(service.document());
        RotationBodyTrajectoryProjectStore::eraseAll(service.document());
        return { true, true, {} };
    }
}
