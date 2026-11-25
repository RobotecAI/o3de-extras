/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Entity.h>
#if !defined(Q_MOC_RUN)
#include <AzCore/Math/Crc.h>
#include <AzCore/std/string/string.h>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QWizardPage>
#include <RobotImporter/Pages/WarningBanner.h>
#endif

namespace ROS2RobotImporter
{
    class RobotImporterWidget;
    class PrefabMakerPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit PrefabMakerPage(RobotImporterWidget* parent);
        void SetProposedPrefabName(const AZStd::string prefabName);
        AZStd::string GetPrefabName() const;
        void ReportProgress(const AZStd::string& progressForUser);

        //! Warns when the model holds more links than a single PhysX articulation supports.
        //! @param linkCount number of links the importer creates for this model
        //! @param useArticulations whether the import uses articulations, which is the only case the limit applies to
        void ReportArticulationLinkCount(size_t linkCount, bool useArticulations);
        void SetSuccess(bool success);
        bool isComplete() const override;
    Q_SIGNALS:
        void onCreateButtonPressed();

    private:
        bool m_success;
        QLineEdit* m_prefabName;
        QPushButton* m_createButton;
        QTextEdit* m_log;
        WarningBanner* m_articulationWarning{};
    };
} // namespace ROS2RobotImporter
