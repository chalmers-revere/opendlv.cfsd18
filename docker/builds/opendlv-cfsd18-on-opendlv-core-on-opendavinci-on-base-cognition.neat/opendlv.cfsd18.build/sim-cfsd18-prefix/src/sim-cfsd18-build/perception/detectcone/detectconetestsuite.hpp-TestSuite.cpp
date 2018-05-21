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
    std::ofstream ofstr("TEST-opendlv-sim-cfsd18-perception-detectcone-detectconetestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-sim-cfsd18-perception-detectcone-detectconetestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_DetectConeTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/sim-cfsd18/perception/detectcone/testsuites/detectconetestsuite.hpp"

static DetectConeTest suite_DetectConeTest;

static CxxTest::List Tests_DetectConeTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_DetectConeTest( "/opt/opendlv.cfsd18.sources/code/sim-cfsd18/perception/detectcone/testsuites/detectconetestsuite.hpp", 26, "DetectConeTest", suite_DetectConeTest, Tests_DetectConeTest );

static class TestDescription_suite_DetectConeTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_DetectConeTest_testApplication() : CxxTest::RealTestDescription( Tests_DetectConeTest, suiteDescription_DetectConeTest, 36, "testApplication" ) {}
 void runTest() { suite_DetectConeTest.testApplication(); }
} testDescription_suite_DetectConeTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
