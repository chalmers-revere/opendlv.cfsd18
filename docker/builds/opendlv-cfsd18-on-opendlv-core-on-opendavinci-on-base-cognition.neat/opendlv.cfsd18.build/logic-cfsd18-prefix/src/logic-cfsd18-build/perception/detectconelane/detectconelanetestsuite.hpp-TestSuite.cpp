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
    std::ofstream ofstr("TEST-opendlv-logic-cfsd18-perception-detectconelane-detectconelanetestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-logic-cfsd18-perception-detectconelane-detectconelanetestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_DetectConeLaneTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/logic-cfsd18/perception/detectconelane/testsuites/detectconelanetestsuite.hpp"

static DetectConeLaneTest suite_DetectConeLaneTest;

static CxxTest::List Tests_DetectConeLaneTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_DetectConeLaneTest( "/opt/opendlv.cfsd18.sources/code/logic-cfsd18/perception/detectconelane/testsuites/detectconelanetestsuite.hpp", 26, "DetectConeLaneTest", suite_DetectConeLaneTest, Tests_DetectConeLaneTest );

static class TestDescription_suite_DetectConeLaneTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_DetectConeLaneTest_testApplication() : CxxTest::RealTestDescription( Tests_DetectConeLaneTest, suiteDescription_DetectConeLaneTest, 36, "testApplication" ) {}
 void runTest() { suite_DetectConeLaneTest.testApplication(); }
} testDescription_suite_DetectConeLaneTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
