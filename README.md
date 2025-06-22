# Tesseract IK-Geo plugin

This is an inverse kinematics plugin for `tesseract_kinematics` that uses the [IK-Geo](https://github.com/rpiRobotics/ik-geo) solver.

This repository is under development, and may be merged into the main `tesseract_kinematics` repository in the future.

## Installation

Clone this repository into your `src` directory for building using `colcon`. It is expected
that the `tesseract` core packages are already cloned into the same `src` directory.

```bash
git clone https://github.com/johnwason/tesseract_ikgeo.git
```

Then, build the package using `colcon`:

```bash
colcon build --packages-select tesseract_ikgeo --cmake-args -DCMAKE_BUILD_TYPE=Debug -DTESSERACT_ENABLE_TESTING=ON
```

## Usage

This package provides a plugin for `tesseract_kinematics` that can be used in the same way as other kinematics plugins.
Plugins are configured using a yaml file that is specified in the environments `srdf` file. Examples of various
configurations using `urdf` and `srdf` files for Tesseract can be found in the [`tesseract_support`](https://github.com/tesseract-robotics/tesseract/tree/master/tesseract_support) directory. This project contains example configurations in the [`urdf`](urdf/) directory.

The example `abb_ird2400_plugins_ikgeo.yaml` file configured for the ABB IRB 2400 robot:

```yaml
kinematic_plugins:
  search_libraries:
    - tesseract_ikgeo_factory
  fwd_kin_plugins:
    manipulator:
      default: KDLFwdKinChain
      plugins:
        KDLFwdKinChain:
          class: KDLFwdKinChainFactory
          config:
            base_link: base_link
            tip_link: tool0
  inv_kin_plugins:
    manipulator:
      default: IKGeoInvKin
      plugins:
        IKGeoInvKin:
          class: IKGeoInvKinFactory
          config:
            base_link: base_link
            tip_link: tool0
            solver: IK_spherical_2_parallel
            params:
              H:
              - x: 0.0
                y: 0.0
                z: 1.0
              - x: 0.0
                y: 1.0
                z: 0.0
              - x: 0.0
                y: 1.0
                z: 0.0
              - x: 0.0
                y: 0.0 
                z: 1.0
              - x: 0.0
                y: 1.0
                z: 0.0
              - x: 0.0
                y: 0.0
                z: 1.0
              P:
              - x: 0.0
                y: 0.0
                z: 0.615
              - x: 0.100
                y: 0.0
                z: 0.0
              - x: 0.0
                y: 0.0
                z: 0.705
              - x: -0.135
                y: 0.0
                z: 0.755
              - x: 0.0
                y: 0.0
                z: 0.0
              - x: 0.0
                y: 0.0
                z: 0.0
              - x: 0.0
                y: 0.0
                z: 0.085
              joint_offsets: [0, 0, -1.57079632679, 0, 0, 0]
```

### Parameters

- `base_link`: The base link of the kinematic chain.
- `tip_link`: The end effector link of the kinematic chain.
- `solver`: The IK-Geo solver to use. This can be one of the solvers defined in the IK-Geo library. The available
   solvers are:
  - IK_2_intersecting
  - IK_2_parallel
  - IK_3_parallel_2_intersecting
  - IK_3_parallel
  - IK_gen_6_dof
  - IK_spherical_2_intersecting
  - IK_spherical_2_parallel
  - IK_spherical
- `params`: The parameters for the solver containing H, P, and joint_offsets. See the IK-Geo documentation for details on how to configure these parameters.

Note that the `tip_link` and `base_link` frames must be aligned when the robot has all zero joint angles to match with the IK-Geo solver's expectations. In ROS, the robot is expected to have the end effector pointing toward `X`, with the TCP frame `Z` pointing out and `X` pointing down. To match this expectation, the IK-Geo parameters should be defined to have the robot TCP pointing "up". To match the URDF format which typically has the tool pointing toward `X`, use the `joint_offset` parameters to match the IK-Geo definition to the URDF definition. In the ABB IRB2400 example, Joint 3 has an offset of -90 degrees to match the IK-Geo parameters to the URDF file.

Currently only 6-DOF solvers are supported. Tesseract has limited support for 7-DOF solvers.

## License

Apache 2.0
