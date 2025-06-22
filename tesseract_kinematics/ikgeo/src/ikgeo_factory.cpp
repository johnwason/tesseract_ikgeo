/**
 * @file ikgeo_factory.h
 * @brief Tesseract IK-Geo Inverse kinematics Factory
 *
 * @author John Wason
 * @date Jun 21, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_kinematics/ikgeo/ikgeo_factory.h>
#include <tesseract_kinematics/ikgeo/ikgeo_inv_kin.h>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>

#include <console_bridge/console.h>

namespace YAML {
template<>
struct convert<Eigen::Vector3d> {
  static bool decode(const Node& node, Eigen::Vector3d& rhs) {
    if(!node.IsMap() || node.size() != 3) return false;
    rhs.x() = node["x"].as<double>();
    rhs.y() = node["y"].as<double>();
    rhs.z() = node["z"].as<double>();
    return true;
  }
};
}

namespace tesseract_kinematics
{
std::unique_ptr<InverseKinematics> IKGeoInvKinFactory::create(const std::string& solver_name,
                                                            const tesseract_scene_graph::SceneGraph& scene_graph,
                                                            const tesseract_scene_graph::SceneState& /*scene_state*/,
                                                            const KinematicsPluginFactory& /*plugin_factory*/,
                                                            const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;
  ikgeo_parameters params;
  tesseract_scene_graph::ShortestPath path;

  try
  {
    if (YAML::Node n = config["base_link"])
      base_link = n.as<std::string>();
    else
      throw std::runtime_error("IKGeoInvKinFactory, missing 'base_link' entry");

    if (YAML::Node n = config["tip_link"])
      tip_link = n.as<std::string>();
    else
      throw std::runtime_error("IKGeoInvKinFactory, missing 'tip_link' entry");
    
    if (YAML::Node n = config["solver"])
    {
      if (n.IsScalar())
      {
        auto solver = n.as<std::string>();
        if (solver == "IK_2_intersecting")
        params.solver_type = ikgeo_solver::IK_2_intersecting;
        else if (solver == "IK_2_parallel")
        params.solver_type = ikgeo_solver::IK_2_parallel;
        else if (solver == "IK_3_parallel_2_intersecting")
        params.solver_type = ikgeo_solver::IK_3_parallel_2_intersecting;
        else if (solver == "IK_3_parallel")
        params.solver_type = ikgeo_solver::IK_3_parallel;
        else if (solver == "IK_gen_6_dof")
        params.solver_type = ikgeo_solver::IK_gen_6_dof;
        else if (solver == "IK_spherical_2_intersecting")
        params.solver_type = ikgeo_solver::IK_spherical_2_intersecting;
        else if (solver == "IK_spherical_2_parallel")
        params.solver_type = ikgeo_solver::IK_spherical_2_parallel;
        else if (solver == "IK_spherical")
        params.solver_type = ikgeo_solver::IK_spherical;
        else
        throw std::runtime_error("IKGeoInvKinFactory, unknown 'solver' type: " + solver);
    }
      else
        throw std::runtime_error("IKGeoInvKinFactory, 'solver_name' should be a string!");
    }

    if (YAML::Node ikgeo_params = config["params"])
    {
      if (YAML::Node n = ikgeo_params["H"])
      {
        auto h = n.as<std::vector<Eigen::Vector3d>>();
        if (h.size() != 7 && h.size() != 6)
          throw std::runtime_error("IKGeoInvKinFactory, 'H' should have six or seven elements!");
        params.H = std::move(h);
      }
      else
      {
        throw std::runtime_error("IKGeoInvKinFactory, 'params' missing 'H' entry");
      }

      if (YAML::Node n = ikgeo_params["P"])
      {
        auto p = n.as<std::vector<Eigen::Vector3d>>();
        if (p.size() != 7 && p.size() != 8)
          throw std::runtime_error("IKGeoInvKinFactory, 'P' should have seven or eight elements!");
        params.P = std::move(p);
      }
      else
      {
        throw std::runtime_error("IKGeoInvKinFactory, 'params' missing 'P' entry");
      }

        if (YAML::Node n = ikgeo_params["joint_offsets"])
        {
            auto joint_offsets = n.as<std::vector<double>>();
            if (joint_offsets.size() != params.H.size())
                throw std::runtime_error("IKGeoInvKinFactory, 'joint_offsets' should have the same number of elements as 'H'!");
            params.joint_offsets = std::move(joint_offsets);
        }
        else
        {
            throw std::runtime_error("IKGeoInvKinFactory, 'params' missing 'joint_offsets' entry");
        }
    }
    else
    {
      throw std::runtime_error("IKGeoInvKinFactory, missing 'params' entry");
    }

    path = scene_graph.getShortestPath(base_link, tip_link);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("IKGeoInvKinFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  return std::make_unique<IKGeo>(params, base_link, tip_link, path.active_joints, solver_name);
}

PLUGIN_ANCHOR_IMPL(IKGeoFactoriesAnchor)

}  // namespace tesseract_kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract_kinematics::IKGeoInvKinFactory, IKGeoInvKinFactory);