#include "RotationBodyWorkflowNavigation.h"

#include "../Models/RotationBodyPlanningTranslations.h"

#include "RobotQtWidgetUtils.h"

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QLabel>
#include <QSizePolicy>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>

#include <array>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        struct WorkflowDescriptor
        {
            const char* id;
            const char* translationKey;
            RotationBodyWorkflow workflow;
        };

        constexpr std::array<WorkflowDescriptor, 2> descriptors{ {
            { "modelTransform", "workflow.model_transform", RotationBodyWorkflow::ModelTransform },
            { "sectionRegion", "workflow.section_region", RotationBodyWorkflow::SectionRegionPlanning }
        } };
    }

    RotationBodyWorkflowNavigation::RotationBodyWorkflowNavigation(QWidget* parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("rotationBodyWorkflowNavigation"));
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        m_titleLabel = robot_qt_viewer::makePanelTitle(QString(), this);
        layout->addWidget(m_titleLabel);

        m_buttonGroup = new QButtonGroup(this);
        m_buttonGroup->setExclusive(true);

        m_modelTransformButton = new QToolButton(this);
        m_modelTransformButton->setObjectName(QStringLiteral("rotationBodyWorkflow.modelTransform"));
        m_modelTransformButton->setCheckable(true);
        m_modelTransformButton->setChecked(true);
        m_modelTransformButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        m_modelTransformButton->setIcon(style()->standardIcon(QStyle::SP_FileDialogDetailedView));
        m_modelTransformButton->setMinimumHeight(48);
        m_modelTransformButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_buttonGroup->addButton(m_modelTransformButton, 0);

        m_sectionRegionButton = new QToolButton(this);
        m_sectionRegionButton->setObjectName(QStringLiteral("rotationBodyWorkflow.sectionRegion"));
        m_sectionRegionButton->setCheckable(true);
        m_sectionRegionButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        m_sectionRegionButton->setIcon(style()->standardIcon(QStyle::SP_FileDialogContentsView));
        m_sectionRegionButton->setMinimumHeight(48);
        m_sectionRegionButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_buttonGroup->addButton(m_sectionRegionButton, 1);

        m_viewTransformButton = new QToolButton(this);
        m_viewTransformButton->setObjectName(
            QStringLiteral("rotationBodyCommand.returnToWorkpiece"));
        m_viewTransformButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        m_viewTransformButton->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
        m_viewTransformButton->setMinimumHeight(48);
        m_viewTransformButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

        auto* buttonRow = new QHBoxLayout();
        buttonRow->setContentsMargins(0, 0, 0, 0);
        buttonRow->setSpacing(4);
        buttonRow->addWidget(m_modelTransformButton, 1);
        buttonRow->addWidget(m_sectionRegionButton, 1);
        buttonRow->addWidget(m_viewTransformButton, 1);
        layout->addLayout(buttonRow);

        connect(
            m_buttonGroup,
            static_cast<void(QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked),
            this,
            [this](int id) {
                const RotationBodyWorkflow workflow = id == 1
                    ? RotationBodyWorkflow::SectionRegionPlanning
                    : RotationBodyWorkflow::ModelTransform;
                if(m_currentWorkflow == workflow) {
                    return;
                }
                m_currentWorkflow = workflow;
                emit workflowChanged(workflow);
            });
        connect(m_viewTransformButton, &QToolButton::clicked,
            this, &RotationBodyWorkflowNavigation::viewTransformRequested);
        retranslate();
    }

    void RotationBodyWorkflowNavigation::setLanguageCode(const QString& languageCode)
    {
        const QString canonical =
            RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode);
        if(m_languageCode == canonical) {
            return;
        }
        m_languageCode = canonical;
        retranslate();
    }

    QString RotationBodyWorkflowNavigation::languageCode() const
    {
        return m_languageCode;
    }

    void RotationBodyWorkflowNavigation::setCurrentWorkflow(RotationBodyWorkflow workflow)
    {
        m_currentWorkflow = workflow;
        if(QToolButton* button = workflow == RotationBodyWorkflow::SectionRegionPlanning
            ? m_sectionRegionButton
            : m_modelTransformButton) {
            button->setChecked(true);
        }
    }

    RotationBodyWorkflow RotationBodyWorkflowNavigation::currentWorkflow() const noexcept
    {
        return m_currentWorkflow;
    }

    int RotationBodyWorkflowNavigation::workflowCount() const noexcept
    {
        return static_cast<int>(descriptors.size());
    }

    QStringList RotationBodyWorkflowNavigation::workflowIds() const
    {
        QStringList result;
        for(const WorkflowDescriptor& descriptor : descriptors) {
            result.push_back(QString::fromLatin1(descriptor.id));
        }
        return result;
    }

    void RotationBodyWorkflowNavigation::setViewTransformEnabled(bool enabled)
    {
        m_viewTransformButton->setEnabled(enabled);
    }

    void RotationBodyWorkflowNavigation::retranslate()
    {
        m_titleLabel->setText(
            RotationBodyPlanningTranslations::text(m_languageCode, "workflow.title"));
        m_modelTransformButton->setText(
            RotationBodyPlanningTranslations::text(
                m_languageCode,
                descriptors[0].translationKey));
        m_sectionRegionButton->setText(
            RotationBodyPlanningTranslations::text(
                m_languageCode,
                descriptors[1].translationKey));
        m_viewTransformButton->setText(
            RotationBodyPlanningTranslations::text(
                m_languageCode,
                "command.return_to_workpiece"));
        m_modelTransformButton->setToolTip(m_modelTransformButton->text());
        m_sectionRegionButton->setToolTip(m_sectionRegionButton->text());
        m_viewTransformButton->setToolTip(
            RotationBodyPlanningTranslations::text(
                m_languageCode,
                "command.return_to_workpiece_tooltip"));
    }
}
