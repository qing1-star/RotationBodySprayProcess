#pragma once

#include <RotationBodyTrajectoryPlanning/Core/PlanningTypes.h>

#include <QString>

namespace smrobot::workbench::spray::rotationbody
{
    class RotationBodyPlanningTranslations final
    {
    public:
        static QString canonicalLanguageCode(const QString& languageCode);
        static QString text(const QString& languageCode, const char* key);
        static QString stageName(
            const QString& languageCode,
            smrobot::spray::rotationbody::PlanningStage stage);
        static QString axisName(
            const QString& languageCode,
            smrobot::spray::rotationbody::SignedAxis axis);
        static QString regionName(
            const QString& languageCode,
            smrobot::spray::rotationbody::RegionLabel label);
        static QString errorText(
            const QString& languageCode,
            smrobot::spray::rotationbody::PlanningErrorCode code);
    };
}
