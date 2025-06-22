#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/ikgeo/ikgeo_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

#include <IK_spherical_2_parallel.h>

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
  
  Kinematics<6, 7> kin;
  for(size_t i=0; i < params_.H.size(); ++i)
  {
    kin.H.col(i) = params_.H[i];
  }
  for(size_t i=0; i < params_.P.size(); ++i)
  {
    kin.P.col(i) = params_.P[i];
  }

  auto res = IK_spherical_2_parallel(tip_link_poses.at(tip_link_name_).rotation(),
                  tip_link_poses.at(tip_link_name_).translation(),
                  kin);
  
  IKSolutions solution_set;

  for (size_t i=0; i< res.q.size(); i++)
  { 
    Eigen::VectorXd res1 = res.q[i];
    assert(res1.size() == 6);  // NOLINT
    for (size_t j=0; j<res1.size(); ++j)
    {
      res1(j) += params_.joint_offsets[j];  // Apply joint offsets
    }

    solution_set.emplace_back(Eigen::Map<Eigen::VectorXd>(res1.data(), static_cast<Eigen::Index>(res1.size())));    
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