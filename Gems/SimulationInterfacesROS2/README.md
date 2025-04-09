## O3DE SimulationInterfacesROS2

This gem provides the necessary tools to control the simulation using via the ROS 2 interface defined in
the [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) standard.

### Overview

The gem consists of:

- 'SimulationInterfacesROS2SystemComponent` - system component that is responsible for creating and managing the
  lifecycle of all ROS 2 handlers (services, actions)
- Handlers for the services and actions defined in
  the [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces)

To check which features are supported so far, make a request to the `simulation_interfaces/srv/GetSimulatorFeatures`
service (by default the service name is `/get_simulation_features`). Numerical codes of the supported features can be
then translated using
the [SimulatorFeatures](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg)
message definition.

### Requirements

The required dependencies for this gem are:

- [ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2) version 3.3.0 or newer
- [SimulationInterfaces Gem]() version 1.0.0 or newer !TODO add the link to the repository
- Environment with [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) sourced

### Level setup

To use the `SimulationInterfacesROS2` gem just add it to the dependency list in the `project.json` file (together with the
gems listed in the Requirements section). Since the whole functionality is implemented in the System Component then
everything should work out of the box.

### Services

| Service Name            | Service Type                                   | Returns                                                                                                                                                                                                                                                                                                                                    |
|-------------------------|------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| delete_entity           | simulation_interfaces/srv/DeleteEntity         | [Result](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg) message indicating the operation status                                                                                                                                                                                                         |
| get_entities            | simulation_interfaces/srv/GetEntities          | [Result](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg) message indicating the operation status, string[] with unique names of all entities matching the filters                                                                                                                                        |
| get_entities_states     | simulation_interfaces/srv/GetEntitiesStates    | [Result](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg) message indicating the operation status, string[] with unique names of all entities matching the filters, [EntityState](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityState.msg)[] with states for these entities |
| get_entity_state        | simulation_interfaces/srv/GetEntityState       | [Result](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg) message indicating the operation status, [EntityState](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityState.msg) entity ground truth state                                                                         |
| get_simulation_features | simulation_interfaces/srv/GetSimulatorFeatures | [SimulatorFeatures]() with supported features                                                                                                                                                                                                                                                                                              |
| get_spawnables          | simulation_interfaces/srv/GetSpawnables        | [Result](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg) message indicating the operation status, [Spawnable](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Spawnable.msg)[] with spawnable objects                                                                              |
| set_entity_state        | simulation_interfaces/srv/SetEntityState       | [Result](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg) message indicating the operation status                                                                                                                                                                                                         |
| spawn_entity            | simulation_interfaces/srv/SpawnEntity          | [Result](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg) message indicating the operation status, string with the spawned entity full name                                                                                                                                                               |

To get the full definition of the services, their structure, types of fields etc. please visit the [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) official repository.

### Actions

| Action Name    | Action Type                                | Feedback                                                                     | Returns                                                                                                                            |
|----------------|--------------------------------------------|------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------|
| simulate_steps | simulation_interfaces/action/SimulateSteps | uint64 with number of completed steps, uint64 with number of steps remaining | [Result](https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg) message indicating the operation status |

To get the full definition of the actions, their structure, types of fields etc. please visit the [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) official repository.
