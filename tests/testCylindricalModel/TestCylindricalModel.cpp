#include <string>
#include "TestCylindricalModel.hpp"
#include "cylindrical_model.h"

using namespace std;

TestCylindricalModel::TestCylindricalModel() : model(1.5, 50, 0.1, 0.03, 0.05, 100, 10000, 0.1) 
{} 


TestCylindricalModel::~TestCylindricalModel() {};

void TestCylindricalModel::SetUp() {
    model.readInCloudData((char*)"table_scene_mug_stereo_textured.pcd");
    model.buildPassThroughFilter();
    model.estimatePointNormals();
    model.segmentPlanar();
    model.segmentCylinder();
}

void TestCylindricalModel::TearDown() {};

//Test nonzero input
TEST_F(TestCylindricalModel, NonEmptyReadTest)
{
    ASSERT_NE(model.cloud->points.size(), 0);
}

//Test nonzero cloud size for planar component
TEST_F(TestCylindricalModel, NonEmptyPlaneCloudTest)
{
    ASSERT_NE(model.cloud_plane->points.size(), 0);
}

//Test nonzero cloud size for cylindrical model
TEST_F(TestCylindricalModel, NonEmptyCylinderCloudTest)
{
    ASSERT_NE(model.cloud_cylinder->points.size(), 0);
}

//Test planar coefficients with those computed by reference
//implementation at build time (binary: cylinder_segmentation_reference)
TEST_F(TestCylindricalModel, CoeffTestPlanar) 
{
    ifstream inFile;
    inFile.open("plane_coefficients_ref");
    vector<float>::iterator ptr;
    float reference_coeff;

    ASSERT_NE(model.coefficients_plane->values.size(), 0);
    for (ptr = model.coefficients_plane->values.begin(); ptr < model.coefficients_plane->values.end(); ptr++) 
    {
	ASSERT_TRUE(inFile >> reference_coeff);
	ASSERT_NEAR(*ptr, reference_coeff, 0.0001);
    }
}

//Test cylinder coefficients with those computed by reference
//implementation at build time (binary: cylinder_segmentation_reference)
TEST_F(TestCylindricalModel, CoeffTestCylinder) 
{
    ifstream inFile;
    inFile.open("cylinder_coefficients_ref");
    vector<float>::iterator ptr;
    float reference_coeff;

    ASSERT_NE(model.coefficients_cylinder->values.size(), 0);
    for (ptr = model.coefficients_cylinder->values.begin(); ptr < model.coefficients_cylinder->values.end(); ptr++) 
    {
	ASSERT_TRUE(inFile >> reference_coeff);
	ASSERT_NEAR(*ptr, reference_coeff, 0.0001);
    }
}
