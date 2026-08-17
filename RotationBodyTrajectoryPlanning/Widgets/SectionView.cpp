#include "SectionView.h"

#include "../Models/RotationBodyPlanningTranslations.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QPalette>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        constexpr double viewMarginPixels = 20.0;

        double niceGridStep(double targetStep)
        {
            if(!std::isfinite(targetStep) || targetStep <= 0.0) {
                return 1.0;
            }
            const double exponent = std::floor(std::log10(targetStep));
            const double base = std::pow(10.0, exponent);
            const double normalized = targetStep / base;
            if(normalized <= 1.0) {
                return base;
            }
            if(normalized <= 2.0) {
                return 2.0 * base;
            }
            if(normalized <= 5.0) {
                return 5.0 * base;
            }
            return 10.0 * base;
        }

    }

    SectionView::SectionView(QWidget* parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("rotationBodySectionView"));
        setMinimumSize(220, 230);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        setFocusPolicy(Qt::StrongFocus);
        setMouseTracking(true);
        setAutoFillBackground(false);
    }

    void SectionView::setLanguageCode(const QString& languageCode)
    {
        const QString canonical =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        if(m_languageCode == canonical) {
            return;
        }
        m_languageCode = canonical;
        update();
    }

    QString SectionView::languageCode() const
    {
        return m_languageCode;
    }

    void SectionView::setSnapshot(const RotationBodySectionViewSnapshot& snapshot)
    {
        const bool geometryChanged = !sameContourGeometry(m_snapshot, snapshot);
        m_snapshot = snapshot;
        if(geometryChanged) {
            m_fitPending = true;
            m_userAdjustedView = false;
        }
        update();
    }

    const RotationBodySectionViewSnapshot& SectionView::snapshot() const noexcept
    {
        return m_snapshot;
    }

    void SectionView::setActiveRegionLabel(domain::RegionLabel label)
    {
        if(m_activeLabel == label) {
            return;
        }
        m_activeLabel = label;
        update();
    }

    domain::RegionLabel SectionView::activeRegionLabel() const noexcept
    {
        return m_activeLabel;
    }

    void SectionView::fitToContent()
    {
        m_fitPending = true;
        m_userAdjustedView = false;
        update();
    }

    double SectionView::zoomFactor() const noexcept
    {
        return m_pixelsPerMeter;
    }

    Eigen::Vector2d SectionView::viewCenterYz() const noexcept
    {
        return m_viewCenterYz;
    }

    QPointF SectionView::domainToViewport(const Eigen::Vector2d& pointYz) const
    {
        const QPointF center(width() * 0.5, height() * 0.5);
        return {
            center.x() + (pointYz.x() - m_viewCenterYz.x()) * m_pixelsPerMeter,
            center.y() - (pointYz.y() - m_viewCenterYz.y()) * m_pixelsPerMeter
        };
    }

    Eigen::Vector2d SectionView::viewportToDomain(const QPointF& point) const
    {
        const QPointF center(width() * 0.5, height() * 0.5);
        return {
            m_viewCenterYz.x() + (point.x() - center.x()) / m_pixelsPerMeter,
            m_viewCenterYz.y() - (point.y() - center.y()) / m_pixelsPerMeter
        };
    }

    QColor SectionView::semanticColor(domain::RegionLabel label)
    {
        switch(label) {
        case domain::RegionLabel::Unclassified: return QColor(156, 103, 112);
        case domain::RegionLabel::ToothTop: return QColor(0, 119, 255);
        case domain::RegionLabel::ToothWall: return QColor(255, 122, 0);
        case domain::RegionLabel::ToothBottom: return QColor(22, 163, 74);
        case domain::RegionLabel::Transition: return QColor(217, 70, 239);
        }
        return QColor(156, 103, 112);
    }

    QColor SectionView::boundaryColor()
    {
        return QColor(255, 193, 7);
    }

    void SectionView::paintEvent(QPaintEvent*)
    {
        if(m_fitPending) {
            updateFitTransform();
        }
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.fillRect(rect(), palette().base());
        drawGrid(painter);
        drawSection(painter);
        drawTrajectoryEndpoints(painter);
        drawSelection(painter);
        painter.setPen(QPen(palette().mid().color(), 1.0));
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(rect().adjusted(0, 0, -1, -1));
    }

    void SectionView::resizeEvent(QResizeEvent* event)
    {
        QWidget::resizeEvent(event);
        if(!m_userAdjustedView) {
            m_fitPending = true;
        }
    }

    void SectionView::wheelEvent(QWheelEvent* event)
    {
        const QPointF cursor = event->posF();
        const Eigen::Vector2d anchoredDomain = viewportToDomain(cursor);
        const double steps = static_cast<double>(event->angleDelta().y()) / 120.0;
        const double scaleFactor = std::pow(1.15, steps);
        m_pixelsPerMeter = std::clamp(m_pixelsPerMeter * scaleFactor, 1.0e-3, 1.0e12);
        const QPointF center(width() * 0.5, height() * 0.5);
        m_viewCenterYz.x() =
            anchoredDomain.x() - (cursor.x() - center.x()) / m_pixelsPerMeter;
        m_viewCenterYz.y() =
            anchoredDomain.y() + (cursor.y() - center.y()) / m_pixelsPerMeter;
        m_fitPending = false;
        m_userAdjustedView = true;
        update();
        event->accept();
    }

    void SectionView::mousePressEvent(QMouseEvent* event)
    {
        setFocus(Qt::MouseFocusReason);
        if(event->button() == Qt::RightButton) {
            m_panning = true;
            m_lastPanPoint = event->localPos();
            setCursor(Qt::ClosedHandCursor);
            event->accept();
            return;
        }
        if(event->button() == Qt::LeftButton && m_snapshot.section && m_snapshot.regions) {
            m_selecting = true;
            m_selectionStart = event->localPos();
            m_selectionCurrent = m_selectionStart;
            event->accept();
            return;
        }
        QWidget::mousePressEvent(event);
    }

    void SectionView::mouseMoveEvent(QMouseEvent* event)
    {
        if(m_panning) {
            const QPointF current = event->localPos();
            const QPointF delta = current - m_lastPanPoint;
            m_viewCenterYz.x() -= delta.x() / m_pixelsPerMeter;
            m_viewCenterYz.y() += delta.y() / m_pixelsPerMeter;
            m_lastPanPoint = current;
            m_fitPending = false;
            m_userAdjustedView = true;
            update();
            event->accept();
            return;
        }
        if(m_selecting) {
            m_selectionCurrent = event->localPos();
            update();
            event->accept();
            return;
        }
        QWidget::mouseMoveEvent(event);
    }

    void SectionView::mouseReleaseEvent(QMouseEvent* event)
    {
        if(event->button() == Qt::RightButton && m_panning) {
            m_panning = false;
            unsetCursor();
            event->accept();
            return;
        }
        if(event->button() == Qt::LeftButton && m_selecting) {
            m_selectionCurrent = event->localPos();
            const QPointF pixelDelta = m_selectionCurrent - m_selectionStart;
            m_selecting = false;
            update();
            if(std::abs(pixelDelta.x()) < 4.0 || std::abs(pixelDelta.y()) < 4.0) {
                event->accept();
                return;
            }
            const Eigen::Vector2d first = viewportToDomain(m_selectionStart);
            const Eigen::Vector2d second = viewportToDomain(m_selectionCurrent);
            const Eigen::Vector2d extent = (second - first).cwiseAbs();
            if(extent.minCoeff() <= domainSelectionTolerance()) {
                event->accept();
                return;
            }
            domain::YzRectangle rectangle;
            rectangle.minimum = first.cwiseMin(second);
            rectangle.maximum = first.cwiseMax(second);
            emit rectangleSelected(rectangle, m_activeLabel);
            event->accept();
            return;
        }
        QWidget::mouseReleaseEvent(event);
    }

    void SectionView::mouseDoubleClickEvent(QMouseEvent* event)
    {
        if(event->button() == Qt::LeftButton) {
            fitToContent();
            event->accept();
            return;
        }
        QWidget::mouseDoubleClickEvent(event);
    }

    void SectionView::keyPressEvent(QKeyEvent* event)
    {
        if(event->key() == Qt::Key_Escape && m_selecting) {
            m_selecting = false;
            update();
            event->accept();
            return;
        }
        QWidget::keyPressEvent(event);
    }

    bool SectionView::sameContourGeometry(
        const RotationBodySectionViewSnapshot& first,
        const RotationBodySectionViewSnapshot& second)
    {
        const auto samePoint = [](const std::optional<Eigen::Vector2d>& lhs,
            const std::optional<Eigen::Vector2d>& rhs) {
            return lhs.has_value() == rhs.has_value() &&
                (!lhs || lhs->isApprox(*rhs, 1.0e-12));
        };
        if(!samePoint(first.trajectoryStartYz, second.trajectoryStartYz) ||
            !samePoint(first.trajectoryEndYz, second.trajectoryEndYz) ||
            first.section.has_value() != second.section.has_value()) {
            return false;
        }
        if(!first.section) {
            return true;
        }
        const auto& firstPoints = first.section->pointsYz;
        const auto& secondPoints = second.section->pointsYz;
        if(firstPoints.size() != secondPoints.size() ||
            first.section->closed != second.section->closed) {
            return false;
        }
        for(std::size_t index = 0; index < firstPoints.size(); ++index) {
            if(!firstPoints[index].isApprox(secondPoints[index], 1.0e-12)) {
                return false;
            }
        }
        return true;
    }

    SectionView::DomainBounds SectionView::contentBounds() const
    {
        DomainBounds bounds;
        const auto includePoint = [&bounds](const Eigen::Vector2d& point) {
            if(!point.allFinite()) {
                return;
            }
            if(!bounds.valid) {
                bounds.minimumY = bounds.maximumY = point.x();
                bounds.minimumZ = bounds.maximumZ = point.y();
                bounds.valid = true;
                return;
            }
            bounds.minimumY = std::min(bounds.minimumY, point.x());
            bounds.maximumY = std::max(bounds.maximumY, point.x());
            bounds.minimumZ = std::min(bounds.minimumZ, point.y());
            bounds.maximumZ = std::max(bounds.maximumZ, point.y());
        };
        if(m_snapshot.section) {
            for(const Eigen::Vector2d& point : m_snapshot.section->pointsYz) {
                includePoint(point);
            }
        }
        if(m_snapshot.boundary) {
            for(const Eigen::Vector2d& point : m_snapshot.boundary->polygonYz) {
                includePoint(point);
            }
        }
        if(m_snapshot.trajectoryStartYz) {
            includePoint(*m_snapshot.trajectoryStartYz);
        }
        if(m_snapshot.trajectoryEndYz) {
            includePoint(*m_snapshot.trajectoryEndYz);
        }
        return bounds;
    }

    void SectionView::updateFitTransform()
    {
        const DomainBounds bounds = contentBounds();
        if(!bounds.valid) {
            m_viewCenterYz.setZero();
            m_pixelsPerMeter = 1000.0;
            m_fitPending = false;
            return;
        }
        const double widthMeters = std::max(bounds.maximumY - bounds.minimumY, 1.0e-9);
        const double heightMeters = std::max(bounds.maximumZ - bounds.minimumZ, 1.0e-9);
        const double availableWidth = std::max(1.0, width() - 2.0 * viewMarginPixels);
        const double availableHeight = std::max(1.0, height() - 2.0 * viewMarginPixels);
        m_viewCenterYz = {
            (bounds.minimumY + bounds.maximumY) * 0.5,
            (bounds.minimumZ + bounds.maximumZ) * 0.5
        };
        m_pixelsPerMeter = std::max(
            1.0e-3,
            std::min(availableWidth / widthMeters, availableHeight / heightMeters));
        m_fitPending = false;
    }

    void SectionView::drawGrid(QPainter& painter) const
    {
        const Eigen::Vector2d topLeft = viewportToDomain(QPointF(0.0, 0.0));
        const Eigen::Vector2d bottomRight =
            viewportToDomain(QPointF(width(), height()));
        const double minimumY = std::min(topLeft.x(), bottomRight.x());
        const double maximumY = std::max(topLeft.x(), bottomRight.x());
        const double minimumZ = std::min(topLeft.y(), bottomRight.y());
        const double maximumZ = std::max(topLeft.y(), bottomRight.y());
        const double step = niceGridStep(60.0 / m_pixelsPerMeter);
        QColor gridColor = palette().mid().color();
        gridColor.setAlpha(75);
        painter.setPen(QPen(gridColor, 1.0));
        const double firstY = std::ceil(minimumY / step) * step;
        for(double y = firstY; y <= maximumY + step * 0.5; y += step) {
            const QPointF point = domainToViewport({ y, 0.0 });
            painter.drawLine(QPointF(point.x(), 0.0), QPointF(point.x(), height()));
        }
        const double firstZ = std::ceil(minimumZ / step) * step;
        for(double z = firstZ; z <= maximumZ + step * 0.5; z += step) {
            const QPointF point = domainToViewport({ 0.0, z });
            painter.drawLine(QPointF(0.0, point.y()), QPointF(width(), point.y()));
        }
        QColor axisColor = palette().text().color();
        axisColor.setAlpha(100);
        painter.setPen(QPen(axisColor, 1.2));
        const QPointF origin = domainToViewport(Eigen::Vector2d::Zero());
        if(origin.x() >= 0.0 && origin.x() <= width()) {
            painter.drawLine(QPointF(origin.x(), 0.0), QPointF(origin.x(), height()));
        }
        if(origin.y() >= 0.0 && origin.y() <= height()) {
            painter.drawLine(QPointF(0.0, origin.y()), QPointF(width(), origin.y()));
        }
    }

    void SectionView::drawSection(QPainter& painter) const
    {
        if(!m_snapshot.section || m_snapshot.section->segmentCount() == 0) {
            painter.setPen(palette().placeholderText().color());
            painter.drawText(
                rect().adjusted(16, 16, -16, -16),
                Qt::AlignCenter | Qt::TextWordWrap,
                RotationBodyPlanningTranslations::text(m_languageCode, "section.empty"));
            return;
        }
        const domain::SectionContour& contour = *m_snapshot.section;
        const bool assignmentsValid =
            m_snapshot.regions && m_snapshot.regions->matches(contour);
        for(std::size_t segment = 0; segment < contour.segmentCount(); ++segment) {
            const std::size_t next = segment + 1 < contour.pointsYz.size()
                ? segment + 1
                : 0;
            const domain::RegionLabel label = assignmentsValid
                ? m_snapshot.regions->segmentLabels[segment]
                : domain::RegionLabel::Unclassified;
            painter.setPen(QPen(semanticColor(label), 2.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter.drawLine(
                domainToViewport(contour.pointsYz[segment]),
                domainToViewport(contour.pointsYz[next]));
        }
        if(m_snapshot.boundary && m_snapshot.boundary->polygonYz.size() >= 2) {
            QPainterPath path;
            path.moveTo(domainToViewport(m_snapshot.boundary->polygonYz.front()));
            for(std::size_t index = 1; index < m_snapshot.boundary->polygonYz.size(); ++index) {
                path.lineTo(domainToViewport(m_snapshot.boundary->polygonYz[index]));
            }
            path.closeSubpath();
            painter.setBrush(Qt::NoBrush);
            painter.setPen(QPen(boundaryColor(), 3.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter.drawPath(path);
        }
    }

    void SectionView::drawTrajectoryEndpoints(QPainter& painter) const
    {
        if(!m_snapshot.trajectoryStartYz || !m_snapshot.trajectoryEndYz) {
            return;
        }
        const QPointF start = domainToViewport(*m_snapshot.trajectoryStartYz);
        const QPointF end = domainToViewport(*m_snapshot.trajectoryEndYz);
        const QColor trajectoryColor(255, 214, 64);
        painter.setPen(QPen(trajectoryColor, 1.8, Qt::DashLine));
        painter.drawLine(start, end);

        painter.setPen(QPen(QColor(28, 32, 36), 1.0));
        painter.setBrush(trajectoryColor);
        painter.drawEllipse(start, 5.5, 5.5);
        painter.drawEllipse(end, 5.5, 5.5);

        QFont labelFont = painter.font();
        labelFont.setBold(true);
        painter.setFont(labelFont);
        painter.setPen(trajectoryColor);
        painter.drawText(start + QPointF(8.0, -8.0), QStringLiteral("A"));
        painter.drawText(end + QPointF(8.0, -8.0), QStringLiteral("B"));
    }

    void SectionView::drawSelection(QPainter& painter) const
    {
        if(!m_selecting) {
            return;
        }
        QColor color = semanticColor(m_activeLabel);
        QColor fill = color;
        fill.setAlpha(42);
        painter.setPen(QPen(color, 1.5, Qt::DashLine));
        painter.setBrush(fill);
        painter.drawRect(QRectF(m_selectionStart, m_selectionCurrent).normalized());
    }

    double SectionView::domainSelectionTolerance() const noexcept
    {
        if(!m_snapshot.section) {
            return 1.0e-12;
        }
        return std::max(1.0e-12, m_snapshot.section->toleranceMeters * 2.0);
    }
}
