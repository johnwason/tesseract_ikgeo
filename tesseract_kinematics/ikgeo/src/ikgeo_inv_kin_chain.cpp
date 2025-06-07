#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/ikgeo/ikgeo_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
IKGeo::IKGeo(ikgeo_parameters params,
                     std::string base_link_name,
                     std::string tip_link_name,
                     std::vector<std::string> joint_names,
                     std::string solver_name)
  : params_(params)
  , base_link_name_(std::move(base_link_name))
  , tip_link_name_(std::move(tip_link_name))
  , joint_names_(std::move(joint_names))
  , solver_name_(std::move(solver_name))
{
  if (joint_names_.size() != 6)
    throw std::runtime_error("IKGeo, only support six joints!");
}

InverseKinematics::UPtr IKGeo::clone() const { return std::make_unique<IKGeo>(*this); }

IKGeo::IKGeo(const IKGeo& other) { *this = other; }

IKGeo& IKGeo::operator=(const IKGeo& other)
{
  base_link_name_ = other.base_link_name_;
  tip_link_name_ = other.tip_link_name_;
  joint_names_ = other.joint_names_;
  params_ = other.params_;
  solver_name_ = other.solver_name_;
  return *this;
}

IKSolutions IKGeo::calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  assert(tip_link_poses.size() == 1);                                                       // NOLINT
  assert(tip_link_poses.find(tip_link_name_) != tip_link_poses.end());                      // NOLINT
  assert(std::abs(1.0 - tip_link_poses.at(tip_link_name_).matrix().determinant()) < 1e-6);  // NOLINT
  
  // NOLINTNEXTLINE
  //opw_kinematics::Solutions<double> sols = opw_kinematics::inverse(params_, tip_link_poses.at(tip_link_name_));

  // Check the output
  IKSolutions solution_set;
  //solution_set.reserve(sols.size());
  //for (auto& sol : sols)
  {
    //if (opw_kinematics::isValid<double>(sol))
     // solution_set.emplace_back(Eigen::Map<Eigen::VectorXd>(sol.data(), static_cast<Eigen::Index>(sol.size())));
  }

  return solution_set;
}

Eigen::Index IKGeo::numJoints() const { return joint_names_.size(); }

std::vector<std::string> IKGeo::getJointNames() const { return joint_names_; }
std::string IKGeo::getBaseLinkName() const { return base_link_name_; }
std::string IKGeo::getWorkingFrame() const { return base_link_name_; }
std::vector<std::string> IKGeo::getTipLinkNames() const { return { tip_link_name_ }; }
std::string IKGeo::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics