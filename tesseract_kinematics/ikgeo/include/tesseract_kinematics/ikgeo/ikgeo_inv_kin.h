#ifndef TESSERACT_KINEMATICS_IKGEO_H
#define TESSERACT_KINEMATICS_IKGEO_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>

namespace tesseract_kinematics
{
static const std::string IKGEO_CHAIN_SOLVER_NAME = "IKGeo";

struct ikgeo_parameters
{
    Eigen::Matrix<double, 3, 7> H;
	Eigen::Matrix<double, 3, 7> P;
};

/**@brief IK-Geo Inverse Kinematics Implementation. */
class IKGeo : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<IKGeo>;
  using ConstPtr = std::shared_ptr<const IKGeo>;
  using UPtr = std::unique_ptr<IKGeo>;
  using ConstUPtr = std::unique_ptr<const IKGeo>;

  ~IKGeo() override = default;
  IKGeo(const IKGeo& other);
  IKGeo& operator=(const IKGeo& other);
  IKGeo(IKGeo&&) = default;
  IKGeo& operator=(IKGeo&&) = default;

  /**
   * @brief Construct IK-Geo Inverse Kinematics
   * @param params ikgeo kinematics parameters
   * @param base_link_name The name of the base link for the kinematic chain
   * @param tip_link_name The name of the tip link for the kinematic chain
   * @param joint_names The joint names for the kinematic chain
   * @param solver_name The solver name of the kinematic chain
   */
  IKGeo(ikgeo_parameters params,
            std::string base_link_name,
            std::string tip_link_name,
            std::vector<std::string> joint_names,
            std::string solver_name = IKGEO_CHAIN_SOLVER_NAME);

  IKSolutions calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const override final;

  Eigen::Index numJoints() const override final;
  std::vector<std::string> getJointNames() const override final;
  std::string getBaseLinkName() const override final;
  std::string getWorkingFrame() const override final;
  std::vector<std::string> getTipLinkNames() const override final;
  std::string getSolverName() const override final;
  InverseKinematics::UPtr clone() const override final;

protected:
  ikgeo_parameters params_; /**< @brief The opw kinematics parameters */
  std::string base_link_name_;                /**< @brief Link name of first link in the kinematic object */
  std::string tip_link_name_;                 /**< @brief Link name of last kink in the kinematic object */
  std::vector<std::string> joint_names_;      /**< @brief Joint names for the kinematic object */
  std::string solver_name_{ IKGEO_CHAIN_SOLVER_NAME }; /**< @brief Name of this solver */
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_IKGEO_H