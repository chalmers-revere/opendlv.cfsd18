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
    std::ofstream ofstr("TEST-opendlv-logic-cfsd18-cognition-track-tracktestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-logic-cfsd18-cognition-track-tracktestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_TrackTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/logic-cfsd18/cognition/track/testsuites/tracktestsuite.hpp"

static TrackTest suite_TrackTest;

static CxxTest::List Tests_TrackTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_TrackTest( "/opt/opendlv.cfsd18.sources/code/logic-cfsd18/cognition/track/testsuites/tracktestsuite.hpp", 26, "TrackTest", suite_TrackTest, Tests_TrackTest );

static class TestDescription_suite_TrackTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_TrackTest_testApplication() : CxxTest::RealTestDescription( Tests_TrackTest, suiteDescription_TrackTest, 36, "testApplication" ) {}
 void runTest() { suite_TrackTest.testApplication(); }
} testDescription_suite_TrackTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
