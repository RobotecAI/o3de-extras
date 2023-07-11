/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ArticulationsMaker.h"
#include "CollidersMaker.h"
#include "InertialsMaker.h"
#include "JointsMaker.h"
#include "VisualsMaker.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>
#include <RobotImporter/Utils/SourceAssetsStorage.h>
#include <sdf/sdf.hh>
#include <urdf_parser/urdf_parser.h>

namespace ROS2::SDFormat
{
    //! Encapsulates constructive mapping of URDF elements to a complete prefab with entities and components
    class PrefabMaker
    {
    public:
        //! Construct PrefabMaker from arguments.
        //! @param modelFilePath path to the source URDF model.
        //! @param model parsed URDF model.
        //! @param modelSDFormat parsed SDFormat model.
        //! @param prefabPath path to the prefab which will be created as a result of import.
        //! @param urdfAssetsMapping prepared mapping of URDF meshes to Assets.
        //! @param useArticulations allows urdfImporter to create PhysXArticulations instead of multiple rigid bodies and joints.
        PrefabMaker(
            const AZStd::string& modelFilePath,
            urdf::ModelInterfaceSharedPtr model,
            const AZStd::shared_ptr<sdf::Root> modelSDFormat,
            AZStd::string prefabPath,
            const AZStd::shared_ptr<Utils::UrdfAssetMap> urdfAssetsMapping,
            bool useArticulations = false);

        ~PrefabMaker() = default;

        //! Create and return a prefab corresponding to the SDFormat model as set through the constructor.
        //! @return result which is either a prefab containing the imported model or an error.
        AzToolsFramework::Prefab::CreatePrefabResult CreatePrefab();

        //! Get path to the prefab resulting from the import.
        //! @return path to the prefab.
        const AZStd::string& GetPrefabPath() const;

        //! Get descriptive status of import.
        //! A string with the status, which can be understood by the user.
        AZStd::string GetStatus();

    private:
        AzToolsFramework::Prefab::PrefabEntityResult AddEntitiesForLink(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId);
        void BuildAssetsForLink(urdf::LinkSharedPtr link);
        void AddRobotControl(AZ::EntityId rootEntityId);
        static void MoveEntityToDefaultSpawnPoint(const AZ::EntityId& rootEntityId);

        urdf::ModelInterfaceSharedPtr m_model;
        AZStd::shared_ptr<sdf::Root> m_modelSDFormat;
        AZStd::string m_prefabPath;
        VisualsMaker m_visualsMaker;
        CollidersMaker m_collidersMaker;
        InertialsMaker m_inertialsMaker;
        JointsMaker m_jointsMaker;
        // TODO: switch to SDFormat::ArticulationsMaker
        ROS2::ArticulationsMaker m_articulationsMaker;

        AZStd::mutex m_statusLock;
        AZStd::multimap<AZStd::string, AZStd::string> m_status;

        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetsMapping;
        bool m_useArticulations{ false };
    };
} // namespace ROS2::SDFormat
