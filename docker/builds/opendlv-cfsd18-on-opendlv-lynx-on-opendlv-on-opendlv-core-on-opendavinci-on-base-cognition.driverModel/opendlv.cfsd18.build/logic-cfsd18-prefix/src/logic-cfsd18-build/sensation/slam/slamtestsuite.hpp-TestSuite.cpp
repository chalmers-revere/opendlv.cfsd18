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
    std::ofstream ofstr("TEST-opendlv-logic-cfsd18-sensation-slam-slamtestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-logic-cfsd18-sensation-slam-slamtestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_SlamTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/logic-cfsd18/sensation/slam/testsuites/slamtestsuite.hpp"

static SlamTest suite_SlamTest;

static CxxTest::List Tests_SlamTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_SlamTest( "/opt/opendlv.cfsd18.sources/code/logic-cfsd18/sensation/slam/testsuites/slamtestsuite.hpp", 26, "SlamTest", suite_SlamTest, Tests_SlamTest );

static class TestDescription_suite_SlamTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SlamTest_testApplication() : CxxTest::RealTestDescription( Tests_SlamTest, suiteDescription_SlamTest, 36, "testApplication" ) {}
 void runTest() { suite_SlamTest.testApplication(); }
} testDescription_suite_SlamTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
