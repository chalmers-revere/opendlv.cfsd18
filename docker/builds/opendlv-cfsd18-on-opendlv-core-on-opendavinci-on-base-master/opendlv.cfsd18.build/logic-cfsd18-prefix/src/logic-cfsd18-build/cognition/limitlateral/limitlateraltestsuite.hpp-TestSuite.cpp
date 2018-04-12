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
    std::ofstream ofstr("TEST-opendlv-logic-cfsd18-cognition-limitlateral-limitlateraltestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-logic-cfsd18-cognition-limitlateral-limitlateraltestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_LimitLateralTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/logic-cfsd18/cognition/limitlateral/testsuites/limitlateraltestsuite.hpp"

static LimitLateralTest suite_LimitLateralTest;

static CxxTest::List Tests_LimitLateralTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_LimitLateralTest( "/opt/opendlv.cfsd18.sources/code/logic-cfsd18/cognition/limitlateral/testsuites/limitlateraltestsuite.hpp", 26, "LimitLateralTest", suite_LimitLateralTest, Tests_LimitLateralTest );

static class TestDescription_suite_LimitLateralTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LimitLateralTest_testApplication() : CxxTest::RealTestDescription( Tests_LimitLateralTest, suiteDescription_LimitLateralTest, 36, "testApplication" ) {}
 void runTest() { suite_LimitLateralTest.testApplication(); }
} testDescription_suite_LimitLateralTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
