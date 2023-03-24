/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AssetDatabase/AssetDatabaseConnection.h>
#include <AssetDatabase/PathOrUuid.h>
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Math/Crc.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>

namespace ROS2::Utils
{
    //! Structure contains essential information about the source and product assets in O3DE.
    //! It is designed to provide necessary information for other classes in URDF converter, eg CollidersMaker or VisualsMaker.
    struct AvailableAsset
    {
        //! Relative path to source asset eg `Assets/foo_robot/meshes/bar_link.dae`.
        AZStd::string m_sourceAssetRelativePath;

        //! Relative path to source asset eg `/home/user/project/Assets/foo_robot/meshes/bar_link.dae`.
        AZStd::string m_sourceAssetGlobalPath;

        //! Relative path to source asset eg `foo_robot/meshes/bar_link.azmodel`.
        AZStd::string m_productAssetRelativePath;

        //! Source GUID of source asset
        AZ::Uuid m_sourceGuid = AZ::Uuid::CreateNull();
    };

    //! The structure contains a mapping between URDF's path to O3DE asset information.
    struct UrdfAsset
    {
        //! Unresolved URDF path to mesh, eg `package://meshes/bar_link.dae`.
        AZStd::string m_urdfPath;

        //! Resolved URDF path, points to the valid mesh in the filestystem, eg `/home/user/ros_ws/src/foo_robot/meshes/bar_link.dae'
        AZStd::string m_resolvedUrdfPath;

        //! Checksum of the file located pointed by `m_resolvedUrdfPath`.
        AZ::Crc32 m_urdfFileCRC;

        //! Found O3DE asset.
        AvailableAsset m_availableAssetInfo;
    };

    /// Type that hold result of mapping from URDF path to asset info
    using UrdfAssetMap = AZStd::unordered_map<AZStd::string, Utils::UrdfAsset>;

    //! Function computes CRC32 on first kilobyte of file.
    AZ::Crc32 GetFileCRC(const AZStd::string& filename);

    //! Compute CRC for every source mesh from the assets catalog.
    //! @returns map where key is crc of source file and value is AvailableAsset.
    AZStd::unordered_map<AZ::Crc32, AvailableAsset> GetInterestingSourceAssetsCRC();

    //! Discover an association between meshes in URDF and O3DE source and product assets.
    //! The @param meshesFilenames contains the list of unresolved URDF filenames that are to be found as assets.
    //! Steps:
    //! - Functions resolves URDF filenames with `ResolveURDFPath`.
    //! - Files pointed by resolved URDF patches have their checksum computed `GetFileCRC`.
    //! - Function scans all available O3DE assets by calling `GetInterestingSourceAssetsCRC`.
    //! - Suitable mapping to the O3DE asset is found by comparing the checksum of the file pointed by the URDF path and source asset.
    //! @param meshesFilenames - list of the unresolved path from the URDF file
    //! @param urdfFilename - filename of URDF file, used for resolvement
    //! @returns a URDF Asset map where the key is unresolved URDF path to AvailableAsset
    UrdfAssetMap FindAssetsForUrdf(const AZStd::unordered_set<AZStd::string>& meshesFilenames, const AZStd::string& urdfFilename);

    //! Helper function that gives product's path from source asset GUID
    //! @param sourceAssetUUID is source asset GUID
    //! @param typeId type of product asset
    //! @returns relative path to product, empty string if product is not found
    AZStd::string GetProductAsset(const AZ::Uuid& sourceAssetUUID, const AZ::TypeId typeId);

    //! Helper function that gives AZ::RPI::ModelAsset product asset from source asset GUID
    //! @param sourceAssetUUID is source asset GUID
    //! @returns relative path to product, empty string if product is not found
    AZStd::string GetModelProductAsset(const AZ::Uuid& sourceAssetUUID);

    //! Helper function that gives PhysX::Pipeline::MeshAsset product asset from source asset GUID
    //! @param sourceAssetUUID is source asset GUID
    //! @returns relative path to product, empty string if product is not found
    AZStd::string GetPhysXMeshProductAsset(const AZ::Uuid& sourceAssetUUID);

    //! Creates side-car file (.assetinfo) that configures scene to generate physx Mesh.
    //! @param sourceAssetPath - global path to source asset
    bool createSceneManifest(const AZStd::string sourceAssetPath, bool collider, bool visual);

    //! A function that copies and prepares meshes that are referenced in URDF.
    //! It resolves every mesh, creates a directory in Project's Asset directory, copies files, and prepares assets info.
    //! Finally, it assembles its results into mapping that allows mapping Urdf's mesh name to the source asset.
    //! @param meshesFilenames - files to copy (as unresolved urdf paths)
    //! @param urdFilename - path to URDF file (as a global path)
    //! @param colliders - files to create collider assetinfo (as unresolved urdf paths)
    //! @param visuals - files to create visual assetinfo (as unresolved urdf paths)
    //! @param fileIO - instance to fileIO class
    //! @returns mapping from unresolved urdf paths to source asset info
    UrdfAssetMap CopyAssetForURDFAndCreateAssetMap(
        const AZStd::unordered_set<AZStd::string>& meshesFilenames,
        const AZStd::string& urdfFilename,
        const AZStd::unordered_set<AZStd::string>& colliders,
        const AZStd::unordered_set<AZStd::string>& visual,
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance());

} // namespace ROS2::Utils
