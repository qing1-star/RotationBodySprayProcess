#pragma once

#include "../Models/RotationBodyPlanningViewModel.h"

#include <QWidget>

class QScrollArea;
class QStackedWidget;
class QPushButton;

namespace smrobot::workbench::spray::rotationbody
{
    class ModelTransformPanel;
    class RotationBodyWorkflowNavigation;
    class SectionRegionPanel;
    class SectionView;

    class RotationBodyPlanningLeftPanel final : public QWidget
    {
        Q_OBJECT

    public:
        explicit RotationBodyPlanningLeftPanel(QWidget* parent = nullptr);

        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setViewModel(const RotationBodyPlanningViewModel& viewModel);
        void setImportDirectory(const QString& directory);
        void setCurrentWorkflow(RotationBodyWorkflow workflow);
        RotationBodyWorkflow currentWorkflow() const noexcept;
        RotationBodyWorkflowNavigation* navigation() const noexcept;
        ModelTransformPanel* modelTransformPanel() const noexcept;
        SectionRegionPanel* sectionRegionPanel() const noexcept;
        SectionView* previewView() const noexcept;
        void setMainViewMode(RotationBodyMainViewMode mode);
        RotationBodyMainViewMode mainViewMode() const noexcept;
        void setActiveRegionLabel(smrobot::spray::rotationbody::RegionLabel label);

    signals:
        void workflowChanged(RotationBodyWorkflow workflow);
        void returnToPlanningWorkpieceRequested();
        void importRequested(
            const QString& sourcePath,
            const RotationBodyImportOptions& options);
        void planningDeltaEdited(
            const smrobot::spray::rotationbody::TransformComponents& components);
        void baseTransformEdited(
            const smrobot::spray::rotationbody::TransformComponents& components);
        void flipRequested();
        void resetRequested();
        void publishFrameChanged(PublishFrame frame);
        void confirmFrameRequested();
        void mainViewModeChanged(RotationBodyMainViewMode mode);
        void extractSectionRequested();
        void recognizeRegionsRequested();
        void activeRegionLabelChanged(smrobot::spray::rotationbody::RegionLabel label);
        void regionRectangleSelected(
            const smrobot::spray::rotationbody::YzRectangle& rectangle,
            smrobot::spray::rotationbody::RegionLabel label);
        void undoRequested();
        void redoRequested();
        void restoreAutomaticRequested();
        void boundaryModeChanged(smrobot::spray::rotationbody::BoundaryMode mode);
        void confirmBoundaryRequested();
        void saveProgressAndExitRequested();
        void discardAndExitRequested();

    private:
        void retranslate();

        QString m_languageCode{ QStringLiteral("en") };
        RotationBodyWorkflow m_currentWorkflow{ RotationBodyWorkflow::ModelTransform };
        RotationBodyWorkflowNavigation* m_navigation{ nullptr };
        QStackedWidget* m_contentStack{ nullptr };
        QScrollArea* m_modelScrollArea{ nullptr };
        QScrollArea* m_sectionScrollArea{ nullptr };
        ModelTransformPanel* m_modelTransformPanel{ nullptr };
        SectionRegionPanel* m_sectionRegionPanel{ nullptr };
        QPushButton* m_saveButton{ nullptr };
        QPushButton* m_discardButton{ nullptr };
    };
}
