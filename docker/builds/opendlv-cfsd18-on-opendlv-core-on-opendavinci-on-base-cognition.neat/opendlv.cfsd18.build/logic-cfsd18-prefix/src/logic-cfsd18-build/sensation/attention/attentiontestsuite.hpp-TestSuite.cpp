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
    std::ofstream ofstr("TEST-opendlv-logic-cfsd18-sensation-attention-attentiontestsuite.hpp.xml");
    CxxTest::XUnitPrinter tmp(ofstr);
    CxxTest::RealWorldDescription::_worldName = "opendlv-logic-cfsd18-sensation-attention-attentiontestsuite.hpp";
    status = CxxTest::Main< CxxTest::XUnitPrinter >( tmp, argc, argv );
    return status;
}
bool suite_AttentionTest_init = false;
#include "../../../../../../opendlv.cfsd18.sources/code/logic-cfsd18/sensation/attention/testsuites/attentiontestsuite.hpp"

static AttentionTest suite_AttentionTest;

static CxxTest::List Tests_AttentionTest = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_AttentionTest( "/opt/opendlv.cfsd18.sources/code/logic-cfsd18/sensation/attention/testsuites/attentiontestsuite.hpp", 26, "AttentionTest", suite_AttentionTest, Tests_AttentionTest );

static class TestDescription_suite_AttentionTest_testApplication : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_AttentionTest_testApplication() : CxxTest::RealTestDescription( Tests_AttentionTest, suiteDescription_AttentionTest, 36, "testApplication" ) {}
 void runTest() { suite_AttentionTest.testApplication(); }
} testDescription_suite_AttentionTest_testApplication;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
