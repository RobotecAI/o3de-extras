
#pragma once

#include <AzCore/Component/Component.h>

#include <${Name}/${Name}Bus.h>

namespace ${SanitizedCppName}
{
    class ${SanitizedCppName}SystemComponent
        : public AZ::Component
        , protected ${SanitizedCppName}RequestBus::Handler
    {
    public:
        AZ_COMPONENT(${SanitizedCppName}SystemComponent, "${SysCompClassId}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ${SanitizedCppName}SystemComponent();
        ~${SanitizedCppName}SystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // ${SanitizedCppName}RequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
