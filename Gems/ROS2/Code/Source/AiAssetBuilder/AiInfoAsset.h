#pragma once

#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Asset/GenericAssetHandler.h>
namespace ROS2
{
    class AiInfoAsset : public AZ::Data::AssetData
    {
    public:
        AZ_CLASS_ALLOCATOR(AiInfoAsset, AZ::SystemAllocator);

        AZ_RTTI(AiInfoAsset, "{548bea17-ce71-4861-a00b-e7f8670ef8d5}", AZ::Data::AssetData);

        static void Reflect(AZ::ReflectContext* context)
        {
            AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
            if (serialize)
            {
                serialize->Class<AiInfoAsset>()
                    ->Field("Name", &AiInfoAsset::m_name)
                    ->Field("Description", &AiInfoAsset::m_description)
                    ->Field("BoundingBox", &AiInfoAsset::m_boundingBox)
                    ->Field("Origin", &AiInfoAsset::m_origin);

                AZ::EditContext* edit = serialize->GetEditContext();
                if (edit)
                {
                    edit->Class<AiInfoAsset>("AiInfoAsset", "Asset for representing for AI agents information")
                        ->DataElement(0, &AiInfoAsset::m_name, "Name", "A descriptive name for the AI agent")
                        ->DataElement(0, &AiInfoAsset::m_description, "Description", "A description of the AI agent")
                        ->DataElement(0, &AiInfoAsset::m_boundingBox, "BoundingBox", "The bounding box of the AI agent")
                        ->DataElement(0, &AiInfoAsset::m_origin, "Origin", "The origin of the AI agent");
                }
            }
        }

        AZStd::string m_name;
        AZStd::string m_description;
        AZ::Aabb m_boundingBox;
        AZ::Transform m_origin;
        static constexpr const char* FileExtension = "ainfo";
        static constexpr const char* AssetGroup = "ai";
    };

    using AiInfoAssetHandler = AzFramework::GenericAssetHandler<AiInfoAsset>;
} // namespace ROS2
