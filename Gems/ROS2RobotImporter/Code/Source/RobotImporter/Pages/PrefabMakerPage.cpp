/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PrefabMakerPage.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/string/string.h>

#include <Source/Articulation.h>

#include <RobotImporter/RobotImporterWidget.h>

namespace ROS2RobotImporter
{
    PrefabMakerPage::PrefabMakerPage([[maybe_unused]] RobotImporterWidget* parent)
        : QWizardPage(parent)
        , m_success(false)
    {
        m_prefabName = new QLineEdit(this);
        m_createButton = new QPushButton(tr("Create Prefab"), this);
        m_log = new QTextEdit(this);
        m_log->acceptRichText();
        m_log->setReadOnly(true);

        m_articulationWarning = new WarningBanner(this);

        setTitle(tr("Prefab creation"));
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_articulationWarning);
        QHBoxLayout* layoutInner = new QHBoxLayout;
        layoutInner->addWidget(m_prefabName);
        layoutInner->addWidget(m_createButton);
        layout->addLayout(layoutInner);
        layout->addWidget(m_log);
        setLayout(layout);
        connect(m_createButton, &QPushButton::pressed, this, &PrefabMakerPage::onCreateButtonPressed);
    }

    void PrefabMakerPage::SetProposedPrefabName(const AZStd::string prefabName)
    {
        m_prefabName->setText(QString::fromUtf8(prefabName.data(), int(prefabName.size())));
    }

    AZStd::string PrefabMakerPage::GetPrefabName() const
    {
        return AZStd::string(m_prefabName->text().toUtf8().constData());
    }

    void PrefabMakerPage::ReportProgress(const AZStd::string& progressForUser)
    {
        m_log->setMarkdown(QString::fromUtf8(progressForUser.data(), int(progressForUser.size())));
    }

    void PrefabMakerPage::ReportArticulationLinkCount(size_t linkCount, bool useArticulations)
    {
        if (!useArticulations || linkCount <= PhysX::MaxArticulationLinks)
        {
            m_articulationWarning->ClearWarning();
            return;
        }

        m_articulationWarning->SetWarning(tr("<b>This robot has more links than a PhysX articulation holds.</b>"
                                             "<p>The model has %1 links and the PhysX limit is %2. The system might be unstable.</p>"
                                             "<p>Import with articulations turned off, or split the robot into several articulations.</p>")
                                              .arg(linkCount)
                                              .arg(PhysX::MaxArticulationLinks));

        AZ_Warning(
            "RobotImporter",
            false,
            "Model has %zu links, which is over the PhysX articulation limit of %zu.",
            linkCount,
            PhysX::MaxArticulationLinks);
    }

    void PrefabMakerPage::SetSuccess(bool success)
    {
        m_success = success;
        emit completeChanged();
    }

    bool PrefabMakerPage::isComplete() const
    {
        return m_success;
    }
} // namespace ROS2RobotImporter
