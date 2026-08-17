#pragma once

#include "../Models/RotationBodyPlanningViewModel.h"

#include <QWidget>

#include <QStringList>

class QButtonGroup;
class QLabel;
class QToolButton;

namespace smrobot::workbench::spray::rotationbody
{
    class RotationBodyWorkflowNavigation final : public QWidget
    {
        Q_OBJECT

    public:
        explicit RotationBodyWorkflowNavigation(QWidget* parent = nullptr);

        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setCurrentWorkflow(RotationBodyWorkflow workflow);
        RotationBodyWorkflow currentWorkflow() const noexcept;
        int workflowCount() const noexcept;
        QStringList workflowIds() const;
        void setViewTransformEnabled(bool enabled);

    signals:
        void workflowChanged(RotationBodyWorkflow workflow);
        void viewTransformRequested();

    private:
        void retranslate();

        QString m_languageCode{ QStringLiteral("en") };
        RotationBodyWorkflow m_currentWorkflow{ RotationBodyWorkflow::ModelTransform };
        QLabel* m_titleLabel{ nullptr };
        QButtonGroup* m_buttonGroup{ nullptr };
        QToolButton* m_modelTransformButton{ nullptr };
        QToolButton* m_sectionRegionButton{ nullptr };
        QToolButton* m_viewTransformButton{ nullptr };
    };
}
