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
    std::ofstream ofstr("TEST-opendlv-logic-cfsd18-cognition-brake-braketestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-logic-cfsd18-cognition-brake-braketestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_BrakeTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/logic-cfsd18/cognition/brake/testsuites/braketestsuite.hpp"

static BrakeTest suite_BrakeTest;

static CxxTest::List Tests_BrakeTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_BrakeTest( "/opt/opendlv.cfsd18.sources/code/logic-cfsd18/cognition/brake/testsuites/braketestsuite.hpp", 26, "BrakeTest", suite_BrakeTest, Tests_BrakeTest );

static class TestDescription_suite_BrakeTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_BrakeTest_testApplication() : CxxTest::RealTestDescription( Tests_BrakeTest, suiteDescription_BrakeTest, 36, "testApplication" ) {}
 void runTest() { suite_BrakeTest.testApplication(); }
} testDescription_suite_BrakeTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
