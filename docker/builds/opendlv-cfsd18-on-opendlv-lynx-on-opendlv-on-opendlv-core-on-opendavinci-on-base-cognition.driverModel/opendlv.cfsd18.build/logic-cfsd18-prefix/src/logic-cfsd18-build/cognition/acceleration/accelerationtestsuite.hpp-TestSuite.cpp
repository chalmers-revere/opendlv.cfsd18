/* Generated file, do not edit */

#ifndef CXXTEST_RUNNING
#define CXXTEST_RUNNING
#endif

#include <fstream>
#define _CXXTEST_HAVE_EH
#include <cxxtest/TestListener.h>
#include <cxxtest/TestTracker.h>
#include <cxxtest/TestRunner.h>
#include <cxxtest/RealDescriptions.h>
#include <cxxtest/TestMain.h>
#include <cxxtest/XUnitPrinter.h>

int main( int argc, char *argv[] ) {
 int status;
    std::ofstream ofstr("TEST-opendlv-logic-cfsd18-cognition-acceleration-accelerationtestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-logic-cfsd18-cognition-acceleration-accelerationtestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_AccelerationTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/logic-cfsd18/cognition/acceleration/testsuites/accelerationtestsuite.hpp"

static AccelerationTest suite_AccelerationTest;

static CxxTest::List Tests_AccelerationTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_AccelerationTest( "/opt/opendlv.cfsd18.sources/code/logic-cfsd18/cognition/acceleration/testsuites/accelerationtestsuite.hpp", 26, "AccelerationTest", suite_AccelerationTest, Tests_AccelerationTest );

static class TestDescription_suite_AccelerationTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_AccelerationTest_testApplication() : CxxTest::RealTestDescription( Tests_AccelerationTest, suiteDescription_AccelerationTest, 36, "testApplication" ) {}
 void runTest() { suite_AccelerationTest.testApplication(); }
} testDescription_suite_AccelerationTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
