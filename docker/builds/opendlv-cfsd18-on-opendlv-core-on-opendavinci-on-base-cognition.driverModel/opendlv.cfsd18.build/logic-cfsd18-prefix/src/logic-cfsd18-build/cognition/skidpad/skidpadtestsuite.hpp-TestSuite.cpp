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
    std::ofstream ofstr("TEST-opendlv-logic-cfsd18-cognition-skidpad-skidpadtestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-logic-cfsd18-cognition-skidpad-skidpadtestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_SkidpadTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/logic-cfsd18/cognition/skidpad/testsuites/skidpadtestsuite.hpp"

static SkidpadTest suite_SkidpadTest;

static CxxTest::List Tests_SkidpadTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_SkidpadTest( "/opt/opendlv.cfsd18.sources/code/logic-cfsd18/cognition/skidpad/testsuites/skidpadtestsuite.hpp", 26, "SkidpadTest", suite_SkidpadTest, Tests_SkidpadTest );

static class TestDescription_suite_SkidpadTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_SkidpadTest_testApplication() : CxxTest::RealTestDescription( Tests_SkidpadTest, suiteDescription_SkidpadTest, 36, "testApplication" ) {}
 void runTest() { suite_SkidpadTest.testApplication(); }
} testDescription_suite_SkidpadTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
