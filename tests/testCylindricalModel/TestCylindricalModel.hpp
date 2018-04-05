#include "gtest/gtest.h"
#include "cylindrical_model.h"

// The fixture for testing class Foo.
class TestCylindricalModel : public ::testing::Test {
    public:
	CylindricalModel model;

    protected:

	//Initialize the CylindricalModel object with parameters for 
	//cylinder segmentation
	TestCylindricalModel();

	virtual ~TestCylindricalModel();

	//Perform the compuatation
	virtual void SetUp();

	virtual void TearDown();
};
