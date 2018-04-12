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
    std::ofstream ofstr("TEST-opendlv-logic-cfsd18-action-lateral-lateraltestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-logic-cfsd18-action-lateral-lateraltestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_LateralTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/logic-cfsd18/action/lateral/testsuites/lateraltestsuite.hpp"

static LateralTest suite_LateralTest;

static CxxTest::List Tests_LateralTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_LateralTest( "/opt/opendlv.cfsd18.sources/code/logic-cfsd18/action/lateral/testsuites/lateraltestsuite.hpp", 26, "LateralTest", suite_LateralTest, Tests_LateralTest );

static class TestDescription_suite_LateralTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LateralTest_testApplication() : CxxTest::RealTestDescription( Tests_LateralTest, suiteDescription_LateralTest, 36, "testApplication" ) {}
 void runTest() { suite_LateralTest.testApplication(); }
} testDescription_suite_LateralTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
