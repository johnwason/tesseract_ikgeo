#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <fstream>
#include <console_bridge/console.h>
#include <filesystem>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/ikgeo/ikgeo_factory.h>
#include <tesseract_environment/environment.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>


TEST(IKGeo, IKGeoInvKin)
{
    tesseract_common::GeneralResourceLocator locator;
    auto env = std::make_shared<tesseract_environment::Environment>();
  std::filesystem::path urdf_path(
          locator.locateResource("package://tesseract_support/urdf/abb_irb2400.urdf")->getFilePath());
      std::filesystem::path srdf_path(
          locator.locateResource("package://tesseract_ikgeo/urdf/abb_irb2400_ikgeo.srdf")->getFilePath());
          

      auto success = env->init(urdf_path, srdf_path, std::make_shared<tesseract_common::GeneralResourceLocator>());
      ASSERT_TRUE(success);
      EXPECT_TRUE(env->getResourceLocator() != nullptr);
      auto kin_group = env->getKinematicGroup("manipulator");
      auto& invkin = kin_group->getInverseKinematics();
      EXPECT_EQ(invkin.getSolverName(), "IKGeoInvKin");

      Eigen::VectorXd test_joints = Eigen::VectorXd::Zero(6);
    test_joints << 0.1, -0.2, 0.03, -0.1, -0.5, 1.0;
    //test_joints << 0.0, 0.0, -1.57079632679, 0.0, 0.0, 0.0;
    auto fwd_kin = kin_group->calcFwdKin(test_joints);
    
    tesseract_common::TransformMap ik_input;
    ik_input["tool0"] = fwd_kin.at("tool0");
    
    auto solutions = invkin.calcInvKin(ik_input, test_joints);
    ASSERT_GT(solutions.size(), 0);

    bool found_solution = false;
    for (const auto& sol : solutions)
    {      
        std::cout << "Solution: " << sol.transpose() << std::endl;
        // find distance between solution and test_joints
        double distance = (sol - test_joints).norm();
        if (distance < 1e-4)
        {
            found_solution = true;
            break;
        }        
    }

    ASSERT_TRUE(found_solution);

      
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Set to debug to also exercies debug code
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  return RUN_ALL_TESTS();
}