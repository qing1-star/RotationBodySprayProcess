#pragma once

#include <string>

namespace simulation_project
{
    class ProjectDocumentService;
}

namespace smrobot::workbench::spray::rotationbody
{
    struct PlanningObjectReplacementResult
    {
        bool success{ false };
        bool changed{ false };
        std::string error;
    };

    class RotationBodyPlanningDocumentOperations
    {
    public:
        static PlanningObjectReplacementResult replacePlanningObject(
            simulation_project::ProjectDocumentService& service,
            const std::string& previousObjectId,
            const std::string& currentObjectId);
    };
}
