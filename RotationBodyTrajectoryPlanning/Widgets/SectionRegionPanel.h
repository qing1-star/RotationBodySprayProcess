#pragma once

#include "../Models/RotationBodyPlanningViewModel.h"

#include <QIcon>
#include <QWidget>

#include <array>

class QButtonGroup;
class QGroupBox;
class QLabel;
class QPushButton;
class QToolButton;

namespace smrobot::workbench::spray::rotationbody
{
    class SectionView;

    class SectionRegionPanel final : public QWidget
    {
        Q_OBJECT

    public:
        explicit SectionRegionPanel(QWidget* parent = nullptr);

        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setViewModel(const RotationBodyPlanningViewModel& viewModel);
        const RotationBodyPlanningViewModel& viewModel() const noexcept;
        void setMainViewMode(RotationBodyMainViewMode mode);
        RotationBodyMainViewMode mainViewMode() const noexcept;
        void setActiveRegionLabel(smrobot::spray::rotationbody::RegionLabel label);
        smrobot::spray::rotationbody::RegionLabel activeRegionLabel() const noexcept;
        SectionView* previewView() const noexcept;

    signals:
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

    private:
        static QIcon swatchIcon(smrobot::spray::rotationbody::RegionLabel label);
        void updateEnabledState();
        void updateDynamicText();
        void retranslate();

        QString m_languageCode{ QStringLiteral("en") };
        RotationBodyPlanningViewModel m_viewModel;
        RotationBodyMainViewMode m_mainViewMode{ RotationBodyMainViewMode::Scene3d };
        smrobot::spray::rotationbody::RegionLabel m_activeLabel{
            smrobot::spray::rotationbody::RegionLabel::ToothTop
        };

        QLabel* m_stageLabel{ nullptr };
        QLabel* m_errorLabel{ nullptr };
        QGroupBox* m_viewGroup{ nullptr };
        QButtonGroup* m_viewButtonGroup{ nullptr };
        QToolButton* m_sceneButton{ nullptr };
        QToolButton* m_sectionButton{ nullptr };
        QGroupBox* m_previewGroup{ nullptr };
        SectionView* m_previewView{ nullptr };
        QPushButton* m_extractButton{ nullptr };

        QGroupBox* m_regionGroup{ nullptr };
        QPushButton* m_recognizeButton{ nullptr };
        QLabel* m_activeLabelCaption{ nullptr };
        QButtonGroup* m_labelButtonGroup{ nullptr };
        std::array<QToolButton*, 5> m_labelButtons{};

        QGroupBox* m_editGroup{ nullptr };
        QToolButton* m_undoButton{ nullptr };
        QToolButton* m_redoButton{ nullptr };
        QPushButton* m_restoreButton{ nullptr };

        QGroupBox* m_boundaryGroup{ nullptr };
        QButtonGroup* m_boundaryButtonGroup{ nullptr };
        QToolButton* m_maximumYButton{ nullptr };
        QToolButton* m_envelopeButton{ nullptr };
        QPushButton* m_confirmBoundaryButton{ nullptr };
    };
}
