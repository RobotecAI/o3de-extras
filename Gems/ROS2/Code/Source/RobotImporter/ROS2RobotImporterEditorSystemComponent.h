/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "Bus/RobotImporterBus.h"
#include "ROS2RobotImporterSystemComponent.h"
#include <AzToolsFramework/Entity/EditorEntityContextBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/string/string.h>

#include <urdf_parser/urdf_parser.h>
#include <RobotImporter/Utils/SourceAssetsStorage.h>
namespace ROS2
{

    enum class UrdfImportState
    {
        In_Progress,
        In_Progress_Copy_Assets,
        In_Progress_Wait_For_AP,
        In_Progress_Create_Prefab,
        Done,
        Failed
    };

    //! Editor component for RobotImporter widget
    class ROS2RobotImporterEditorSystemComponent
        : public ROS2RobotImporterSystemComponent
        , private AzToolsFramework::EditorEvents::Bus::Handler
        , private RobotImporterRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2RobotImporterEditorSystemComponent, "{1cc069d0-72f9-411e-a94b-9159979e5a0c}", ROS2RobotImporterSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        ROS2RobotImporterEditorSystemComponent() = default;
        ~ROS2RobotImporterEditorSystemComponent() = default;

    private:
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // AzToolsFramework::EditorEvents::Bus::Handler overrides
        void NotifyRegisterViews() override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // RobotImporterRequestsBus::Handler overrides
        bool GeneratePrefabFromFile(const AZStd::string_view filePath, bool importAssetWithUrdf, bool useArticulation) override;
        //////////////////////////////////////////////////////////////////////////

        bool SynchonousGeneratePrefabFromFile();
        AZStd::string m_importedPath;
        bool m_importAssetWithUrdf = false;
        bool m_useArticulation = false;
        UrdfImportState m_importState = UrdfImportState::Done;
        urdf::ModelInterfaceSharedPtr m_parsedUrdf;
        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetsMapping;
    };
} // namespace ROS2
