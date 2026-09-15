#pragma once

#include "RobotQtViewerViewportServices.h"

#include <QMap>

class FakeRotationBodyViewportServices final
    : public robot_qt_viewer::RobotQtViewerViewportServices
{
public:
    QMap<QString, QMap<QString, robot_qt_viewer::ViewportLineOverlay>> overlays;
    QStringList clearedOwners;
    QString previewedObjectId;
    simulation_project::TransformDesc previewedObjectTransform;
    QString focusedObjectId;
    robot_qt_viewer::RobotQtViewerObjectFocusView focusedObjectView{
        robot_qt_viewer::RobotQtViewerObjectFocusView::Front
    };
    int upsertOverlayCalls{ 0 };
    int clearOwnerCalls{ 0 };
    int previewObjectCalls{ 0 };
    int focusObjectCalls{ 0 };
    int clearObjectFocusCalls{ 0 };
    int loadProjectCalls{ 0 };
    bool nextLoadSucceeds{ true };
    bool rejectOverlayUpserts{ false };
    QString loadFailureMessage{ QStringLiteral("Injected viewport reload failure.") };
    QString overlayFailureMessage{ QStringLiteral("Injected overlay upsert failure.") };

    const robot_qt_viewer::ViewportLineOverlay* overlay(
        const QString& ownerId,
        const QString& overlayId) const
    {
        const auto owner = overlays.constFind(ownerId);
        if(owner == overlays.cend()) {
            return nullptr;
        }
        const auto found = owner->constFind(overlayId);
        return found == owner->cend() ? nullptr : &found.value();
    }

    bool upsertLineOverlay(
        const robot_qt_viewer::ViewportLineOverlay& value,
        QString* errorMessage = nullptr) override
    {
        if(!robot_qt_viewer::validateViewportLineOverlay(value, errorMessage)) {
            return false;
        }
        if(rejectOverlayUpserts) {
            if(errorMessage != nullptr) {
                *errorMessage = overlayFailureMessage;
            }
            return false;
        }
        overlays[value.ownerId][value.overlayId] = value;
        ++upsertOverlayCalls;
        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }

    bool clearLineOverlay(
        const QString& ownerId,
        const QString& overlayId,
        QString* errorMessage = nullptr) override
    {
        if(!robot_qt_viewer::validateViewportLineOverlayId(
            ownerId,
            overlayId,
            errorMessage)) {
            return false;
        }
        auto owner = overlays.find(ownerId);
        if(owner != overlays.end()) {
            owner->remove(overlayId);
            if(owner->isEmpty()) {
                overlays.erase(owner);
            }
        }
        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }

    bool clearLineOverlaysByOwner(
        const QString& ownerId,
        QString* errorMessage = nullptr) override
    {
        if(ownerId.trimmed().isEmpty()) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("Viewport line overlay owner ID must not be empty.");
            }
            return false;
        }
        overlays.remove(ownerId);
        clearedOwners.push_back(ownerId);
        ++clearOwnerCalls;
        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }

    void selectRobotMount(const QString&, const QString&, const QString&) override {}
    bool setActivePreviewRobotMount(const QString&) override { return false; }
    bool previewRobotMountTransform(
        const QString&,
        const simulation_project::TransformDesc&) override { return false; }
    bool previewRobotMountLink(const QString&, const QString&) override { return false; }
    bool upsertPreviewRobotMount(const simulation_project::RobotMountDesc&) override { return false; }
    bool removePreviewRobotMount(const QString&) override { return false; }
    void previewRobotBaseTransform(
        const QString&,
        const simulation_project::TransformDesc&) override {}
    void previewSceneObjectTransform(
        const QString& objectId,
        const simulation_project::TransformDesc& transform) override
    {
        previewedObjectId = objectId;
        previewedObjectTransform = transform;
        ++previewObjectCalls;
    }
    bool commitSceneObjectTransform(
        const QString&,
        const simulation_project::TransformDesc&) override { return false; }
    bool removeSceneObject(const QString&) override { return false; }
    void selectObjectFrame(const QString&, const QString&) override {}
    bool previewObjectFrameTransform(
        const QString&,
        const QString&,
        const simulation_project::TransformDesc&) override { return false; }
    bool upsertPreviewObjectFrame(
        const QString&,
        const simulation_project::ObjectFrameDesc&) override { return false; }
    void selectRobotLink(const QString&, const QString&) override {}
    void selectRobotJointFrame(const QString&, const QString&) override {}
    void setActiveToolFrameRobot(const QString&) override {}
    void selectSceneObject(const QString&) override {}
    void selectMountedAttachment(const QString&) override {}
    bool setActiveMountedAttachment(const QString&) override { return false; }
    void setToolFrameVisibility(
        const robot_qt_viewer::RobotQtViewerToolFrameVisibility&) override {}
    void setRobotMountFrameVisibility(bool, bool) override {}
    void setPinnedRobotMountFrames(const QStringList&) override {}
    void focusMountFrameLink(const QString&, const QString&) override {}
    void clearMountFrameLinkFocus() override {}
    void focusObjectFrameObject(const QString& objectId) override
    {
        focusedObjectId = objectId;
        focusedObjectView = robot_qt_viewer::RobotQtViewerObjectFocusView::Front;
        ++focusObjectCalls;
    }
    void focusObjectFrameObject(
        const QString& objectId,
        robot_qt_viewer::RobotQtViewerObjectFocusView view) override
    {
        focusedObjectId = objectId;
        focusedObjectView = view;
        ++focusObjectCalls;
    }
    void clearObjectFrameObjectFocus() override
    {
        focusedObjectId.clear();
        ++clearObjectFocusCalls;
    }
    void focusMountedAttachment(const QString&) override {}
    void clearMountedAttachmentFocus() override {}
    void previewObjectCollisionModelVariant(const QString&, const QString&) override {}
    void clearObjectCollisionModelVariantPreview() override {}
    void previewCollisionPairTargets(
        const QString&,
        const QString&,
        const QString&,
        const QString&,
        const QString&,
        const QString&,
        const QString&,
        const QString&) override {}
    robot_qt_viewer::RobotQtViewerViewportLoadResult loadProjectDocument(
        const simulation_project::ProjectDocument&,
        const std::filesystem::path&) override
    {
        ++loadProjectCalls;
        if(!nextLoadSucceeds) {
            nextLoadSucceeds = true;
            return { false, loadFailureMessage };
        }
        overlays.clear();
        previewedObjectId.clear();
        previewedObjectTransform = {};
        focusedObjectId.clear();
        focusedObjectView = robot_qt_viewer::RobotQtViewerObjectFocusView::Front;
        return { true, {} };
    }
    bool refreshCollisionConfiguration(
        const simulation_project::ProjectDocument&,
        const std::filesystem::path&) override { return true; }
    void setCollisionGeometryVisible(bool) override {}
    bool collisionQueriesEnabled() const override { return false; }
    bool setCollisionQueriesEnabled(bool) override { return false; }
    bool setActiveCollisionDetector(const QString&) override { return false; }
    bool setCollisionDetectorEnabled(const QString&, bool) override { return false; }
    bool setCollisionDetectorVisible(const QString&, bool) override { return false; }
    bool updateCollisionDetectorRuntimeOptions(
        const simulation_project::CollisionDetectorDesc&) override { return false; }
    bool rebuildCollisionDetectorsFromDocument(
        const simulation_project::ProjectDocument&) override { return false; }
    bool removeCollisionDetector(const QString&) override { return false; }
    bool setVisibleRobotCollisionVariant(
        const QString&,
        const QString&,
        const QString&) override { return false; }
    QString visibleRobotCollisionVariant(
        const QString&,
        const QString&) const override { return {}; }
    robot_qt_viewer::CollisionRuntimeRobotSummary robotCollisionSummary(
        const QString&) const override { return {}; }
    std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>
    collisionRuntimeDetectors() const override { return {}; }
    bool generateRobotCollisionProxies(
        const QString&,
        const QString&,
        const robot_qt_viewer::CollisionRuntimeProxyRequest&,
        std::vector<simulation_project::CollisionElementOverrideDesc>&) const override
    {
        return false;
    }
    bool generateRobotCollisionProxiesFromExistingCollision(
        const QString&,
        const QString&,
        const robot_qt_viewer::CollisionRuntimeProxyRequest&,
        std::vector<simulation_project::CollisionElementOverrideDesc>&) const override
    {
        return false;
    }
    bool generateRobotCollisionProxiesFromExistingCollision(
        const QString&,
        const robot_qt_viewer::CollisionRuntimeProxyRequest&,
        std::vector<simulation_project::CollisionElementOverrideDesc>&) const override
    {
        return false;
    }
    bool generateRobotCollisionCoacdFromVisual(
        const QString&,
        const QString&,
        std::vector<simulation_project::CollisionElementOverrideDesc>&) const override
    {
        return false;
    }
    bool generateRobotCollisionCoacdFromExistingCollision(
        const QString&,
        const QString&,
        std::vector<simulation_project::CollisionElementOverrideDesc>&) const override
    {
        return false;
    }
    bool generateObjectCollisionCoacdFromVisual(
        const QString&,
        std::vector<simulation_project::ObjectCollisionElementOverrideDesc>&) const override
    {
        return false;
    }
    bool generateMissingRobotCollisionProxies(
        const QString&,
        const robot_qt_viewer::CollisionRuntimeProxyRequest&,
        std::vector<simulation_project::CollisionElementOverrideDesc>&) const override
    {
        return false;
    }
    bool evaluateRobotCollisionProxyQuality(
        const QString&,
        const QString&,
        const robot_qt_viewer::CollisionRuntimeProxyRequest&,
        const std::vector<simulation_project::CollisionElementOverrideDesc>&,
        bool,
        robot_qt_viewer::CollisionRuntimeProxyQualitySummary&) const override
    {
        return false;
    }
    double robotJointValue(const QString&, const QString&, bool* ok = nullptr) const override
    {
        if(ok != nullptr) {
            *ok = false;
        }
        return 0.0;
    }
    void setRobotJointValue(const QString&, const QString&, double) override {}
    void setRobotAutoMotion(const QString&, bool, double, double) override {}
};
