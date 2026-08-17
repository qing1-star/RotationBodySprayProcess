#pragma once

#include "../Models/RotationBodyPlanningViewModel.h"

#include <RotationBodyTrajectoryPlanning/Calibration/WorkpieceCalibration.h>

#include <QWidget>

#include <array>
#include <optional>

class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTabWidget;
class QTableWidget;

namespace smrobot::workbench::spray::rotationbody
{
    class WorkpieceCalibrationPanel final : public QWidget
    {
        Q_OBJECT

    public:
        explicit WorkpieceCalibrationPanel(QWidget* parent = nullptr);

        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setViewModel(const RotationBodyPlanningViewModel& viewModel);
        const std::optional<domain::CalibrationAxisFit>& cylinderFit() const noexcept;
        const std::optional<domain::CalibrationAxisFit>& circleFit() const noexcept;
        const std::optional<domain::WorkpieceFrameCalibration>& frameResult() const noexcept;

    signals:
        void workspaceEdited(
            const smrobot::workbench::spray::rotationbody::WorkpieceCalibrationWorkspace& workspace);
        void baseTransformCalculated(
            const smrobot::spray::rotationbody::TransformComponents& components);

    private:
        domain::CalibrationMode activeMode() const noexcept;
        QTableWidget* tableForMode(domain::CalibrationMode mode) const noexcept;
        std::optional<domain::CalibrationPointList> readPoints(
            QTableWidget* table,
            QString& errorText) const;
        std::optional<Eigen::Vector3d> readReferencePoint(
            const std::array<QLineEdit*, 3>& fields,
            const char* missingKey,
            QString& errorText) const;
        std::optional<domain::CalibrationAxisFit>* fitForMode(
            domain::CalibrationMode mode) noexcept;
        const std::optional<domain::CalibrationAxisFit>* fitForMode(
            domain::CalibrationMode mode) const noexcept;
        const domain::CalibrationAxisFit* selectedAxisFit() const noexcept;
        WorkpieceCalibrationWorkspace workspaceFromInputs() const;
        void applyWorkspace(const WorkpieceCalibrationWorkspace& workspace);
        void emitWorkspaceEdited();

        void runFit(domain::CalibrationMode mode);
        void runActiveFit();
        void runBothFits();
        void clearSelectedPoints();
        void clearActiveMode();
        void invalidateFit(domain::CalibrationMode mode);
        void invalidateFrameResult();
        void calculateAndApply();
        void updateFitResults();
        void updateAxisStatus();
        void updatePoseResult();
        void updateEnabledState();
        void retranslate();
        QString vectorMillimeters(const Eigen::Vector3d& meters, int precision = 3) const;
        QString vectorUnitless(const Eigen::Vector3d& value, int precision = 6) const;

        QString m_languageCode{ QStringLiteral("en") };
        RotationBodyPlanningViewModel m_viewModel;
        bool m_updating{ false };
        bool m_heightInitializedFromModel{ false };
        std::optional<domain::CalibrationAxisFit> m_cylinderFit;
        std::optional<domain::CalibrationAxisFit> m_circleFit;
        std::optional<domain::WorkpieceFrameCalibration> m_frameResult;

        QLabel* m_titleLabel{ nullptr };
        QLabel* m_introLabel{ nullptr };
        QTabWidget* m_modeTabs{ nullptr };
        QWidget* m_cylinderTab{ nullptr };
        QWidget* m_circleTab{ nullptr };
        QLabel* m_cylinderHintLabel{ nullptr };
        QLabel* m_circleHintLabel{ nullptr };
        QTableWidget* m_cylinderTable{ nullptr };
        QTableWidget* m_circleTable{ nullptr };
        QGroupBox* m_operationsGroup{ nullptr };
        QPushButton* m_clearSelectedButton{ nullptr };
        QPushButton* m_clearModeButton{ nullptr };
        QPushButton* m_fitCurrentButton{ nullptr };
        QPushButton* m_fitBothButton{ nullptr };
        QGroupBox* m_fitResultGroup{ nullptr };
        QLabel* m_cylinderResultLabel{ nullptr };
        QLabel* m_circleResultLabel{ nullptr };
        QCheckBox* m_showCylinderCheck{ nullptr };
        QCheckBox* m_showCircleCheck{ nullptr };

        QGroupBox* m_frameGroup{ nullptr };
        QLabel* m_axisSourceLabel{ nullptr };
        QComboBox* m_axisSourceCombo{ nullptr };
        QLabel* m_axisStatusLabel{ nullptr };
        QLabel* m_referenceHeaderLabel{ nullptr };
        std::array<QLabel*, 3> m_referenceRowLabels{};
        std::array<QLabel*, 3> m_referenceColumnLabels{};
        std::array<QLineEdit*, 3> m_topReferenceFields{};
        std::array<QLineEdit*, 3> m_yStartFields{};
        std::array<QLineEdit*, 3> m_yEndFields{};
        QLabel* m_heightLabel{ nullptr };
        QDoubleSpinBox* m_heightSpin{ nullptr };
        QPushButton* m_calculateButton{ nullptr };
        QLabel* m_poseResultLabel{ nullptr };
    };
}
