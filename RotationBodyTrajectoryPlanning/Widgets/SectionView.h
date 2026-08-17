#pragma once

#include "../Models/RotationBodyPlanningViewModel.h"

#include <QColor>
#include <QPointF>
#include <QWidget>

#include <Eigen/Core>

class QKeyEvent;
class QMouseEvent;
class QPaintEvent;
class QPainter;
class QResizeEvent;
class QWheelEvent;

namespace smrobot::workbench::spray::rotationbody
{
    class SectionView final : public QWidget
    {
        Q_OBJECT

    public:
        explicit SectionView(QWidget* parent = nullptr);

        void setLanguageCode(const QString& languageCode);
        QString languageCode() const;
        void setSnapshot(const RotationBodySectionViewSnapshot& snapshot);
        const RotationBodySectionViewSnapshot& snapshot() const noexcept;
        void setActiveRegionLabel(smrobot::spray::rotationbody::RegionLabel label);
        smrobot::spray::rotationbody::RegionLabel activeRegionLabel() const noexcept;
        void fitToContent();

        double zoomFactor() const noexcept;
        Eigen::Vector2d viewCenterYz() const noexcept;
        QPointF domainToViewport(const Eigen::Vector2d& pointYz) const;
        Eigen::Vector2d viewportToDomain(const QPointF& point) const;

        static QColor semanticColor(smrobot::spray::rotationbody::RegionLabel label);
        static QColor boundaryColor();

    signals:
        void rectangleSelected(
            const smrobot::spray::rotationbody::YzRectangle& rectangle,
            smrobot::spray::rotationbody::RegionLabel label);

    protected:
        void paintEvent(QPaintEvent* event) override;
        void resizeEvent(QResizeEvent* event) override;
        void wheelEvent(QWheelEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;
        void mouseDoubleClickEvent(QMouseEvent* event) override;
        void keyPressEvent(QKeyEvent* event) override;

    private:
        struct DomainBounds
        {
            double minimumY{ 0.0 };
            double maximumY{ 0.0 };
            double minimumZ{ 0.0 };
            double maximumZ{ 0.0 };
            bool valid{ false };
        };

        static bool sameContourGeometry(
            const RotationBodySectionViewSnapshot& first,
            const RotationBodySectionViewSnapshot& second);
        DomainBounds contentBounds() const;
        void updateFitTransform();
        void drawGrid(QPainter& painter) const;
        void drawSection(QPainter& painter) const;
        void drawTrajectoryEndpoints(QPainter& painter) const;
        void drawSelection(QPainter& painter) const;
        double domainSelectionTolerance() const noexcept;

        QString m_languageCode{ QStringLiteral("en") };
        RotationBodySectionViewSnapshot m_snapshot;
        smrobot::spray::rotationbody::RegionLabel m_activeLabel{
            smrobot::spray::rotationbody::RegionLabel::ToothTop
        };
        Eigen::Vector2d m_viewCenterYz = Eigen::Vector2d::Zero();
        double m_pixelsPerMeter{ 1000.0 };
        bool m_fitPending{ true };
        bool m_userAdjustedView{ false };
        bool m_selecting{ false };
        bool m_panning{ false };
        QPointF m_selectionStart;
        QPointF m_selectionCurrent;
        QPointF m_lastPanPoint;
    };
}
